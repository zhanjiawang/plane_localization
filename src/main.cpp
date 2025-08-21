#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/MarkerArray.h>

#include <atomic>
#include <deque>
#include <fstream>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>

#include "CustomMsg.h"
#include "plane_octree.hpp"

typedef struct ScoreIndex {
  int index;
  float score;
} ScoreIndex;

typedef struct PlaneSelectRule {
  int max_number;
  int min_area;
  float min_angle;
  float max_angle;
} PlaneSelectRule;

bool get_test_data_ = false;
int test_index_;

std::string current_path_;
std::string relative_path_;
float lidar_max_range_;
std::vector<PlaneSelectRule> map_plane_select_rules_;
std::vector<PlaneSelectRule> scan_plane_select_rules_;
float select_plane_angel_threshold_;
float second_third_plane_angel_threshold_;
float translation_radius_;
float rotation_radius_;
float coplanar_angel_weight_;
float coplanar_distance_weight_;
float coplanar_attenuation_coefficient_;
int verify_number_;

ros::Subscriber sub_lidar_cloud_;

ros::Publisher pub_map_points_;
ros::Publisher pub_scan_points_;
ros::Publisher pub_plane_point_cloud_map_;
ros::Publisher pub_plane_normal_map_;
ros::Publisher pub_plane_point_cloud_scan_;
ros::Publisher pub_plane_normal_scan_;
ros::Publisher pub_point_cloud_occupy_map_;
ros::Publisher pub_transformation_matrix_points_all_;
ros::Publisher pub_transformation_matrix_points_good_;

std::vector<Eigen::Matrix4f> pose_matrix_vector_;

int plane_octree_occupy_map_depth_;
std::shared_ptr<PlaneOctree> plane_octree_occupy_map_;

std::vector<std::shared_ptr<OctreePlane>> octree_planes_map_;
std::vector<std::shared_ptr<OctreePlane>> octree_planes_map_select_;
std::vector<std::vector<int>> octree_planes_map_select_index_;

//读取参数
void ReadParam(ros::NodeHandle &nh) {
  nh.param<int>("plane_localization/test_index", test_index_, 0);
  std::cout << "test_index: " << test_index_ << std::endl;

  nh.param<std::string>("plane_localization/relative_path", relative_path_, "");
  std::cout << "relative_path: " << relative_path_ << std::endl;
  nh.param<float>("plane_localization/lidar_max_range", lidar_max_range_, 50.0);
  std::cout << "lidar_max_range: " << lidar_max_range_ << std::endl;
  std::vector<float> map_plane_select_rules;
  nh.getParam("plane_localization/map_plane_select_rules",
              map_plane_select_rules);
  int map_plane_select_rules_num = map_plane_select_rules.size() / 4;
  for (int i = 0; i < map_plane_select_rules_num; i++) {
    PlaneSelectRule plane_slect_rule;
    plane_slect_rule.max_number = map_plane_select_rules[i * 4];
    plane_slect_rule.min_area = map_plane_select_rules[i * 4 + 1];
    plane_slect_rule.min_angle = map_plane_select_rules[i * 4 + 2];
    plane_slect_rule.max_angle = map_plane_select_rules[i * 4 + 3];
    map_plane_select_rules_.push_back(plane_slect_rule);
  }
  std::cout << "map_plane_select_rules: " << std::endl;
  for (int i = 0; i < map_plane_select_rules_.size(); i++) {
    std::cout << map_plane_select_rules_[i].max_number << " "
              << map_plane_select_rules_[i].min_area << " "
              << map_plane_select_rules_[i].min_angle << " "
              << map_plane_select_rules_[i].max_angle << std::endl;
  }
  std::vector<float> scan_plane_select_rules;
  nh.getParam("plane_localization/scan_plane_select_rules",
              scan_plane_select_rules);
  int scan_plane_select_rules_num = scan_plane_select_rules.size() / 4;
  for (int i = 0; i < scan_plane_select_rules_num; i++) {
    PlaneSelectRule plane_slect_rule;
    plane_slect_rule.max_number = scan_plane_select_rules[i * 4];
    plane_slect_rule.min_area = scan_plane_select_rules[i * 4 + 1];
    plane_slect_rule.min_angle = scan_plane_select_rules[i * 4 + 2];
    plane_slect_rule.max_angle = scan_plane_select_rules[i * 4 + 3];
    scan_plane_select_rules_.push_back(plane_slect_rule);
  }
  std::cout << "scan_plane_select_rules: " << std::endl;
  for (int i = 0; i < scan_plane_select_rules_.size(); i++) {
    std::cout << scan_plane_select_rules_[i].max_number << " "
              << scan_plane_select_rules_[i].min_area << " "
              << scan_plane_select_rules_[i].min_angle << " "
              << scan_plane_select_rules_[i].max_angle << std::endl;
  }
  nh.param<float>("plane_localization/select_plane_angel_threshold",
                  select_plane_angel_threshold_, 4.0);
  std::cout << "select_plane_angel_threshold: " << select_plane_angel_threshold_
            << std::endl;
  nh.param<float>("plane_localization/second_third_plane_angel_threshold",
                  second_third_plane_angel_threshold_, 45.0);
  std::cout << "second_third_plane_angel_threshold: "
            << second_third_plane_angel_threshold_ << std::endl;
  nh.param<float>("plane_localization/translation_radius", translation_radius_,
                  0.25);
  std::cout << "translation_radius: " << translation_radius_ << std::endl;
  nh.param<float>("plane_localization/rotation_radius", rotation_radius_, 4.0);
  std::cout << "rotation_radius: " << rotation_radius_ << std::endl;
  //角度转为弧度
  rotation_radius_ = M_PI * rotation_radius_ / 180.0;
  nh.param<float>("plane_localization/coplanar_angel_weight",
                  coplanar_angel_weight_, 0.3);
  std::cout << "coplanar_angel_weight: " << coplanar_angel_weight_ << std::endl;
  nh.param<float>("plane_localization/coplanar_distance_weight",
                  coplanar_distance_weight_, 0.6);
  std::cout << "coplanar_distance_weight: " << coplanar_distance_weight_
            << std::endl;
  nh.param<float>("plane_localization/coplanar_attenuation_coefficient",
                  coplanar_attenuation_coefficient_, 0.3);
  std::cout << "coplanar_attenuation_coefficient: "
            << coplanar_attenuation_coefficient_ << std::endl;
  nh.param<int>("plane_localization/verify_number", verify_number_, 50);
  std::cout << "verify_number: " << verify_number_ << std::endl;
}

//读取构成地图的关键帧的位置，并估计八叉树地图的跨度
void ReadPose(std::string &path, float &range) {
  std::ifstream pose_file(path);
  if (pose_file.is_open()) {
    std::string line;
    while (getline(pose_file, line)) {
      if (line.length() > 6) {
        float x;
        float y;
        float z;
        float qw;
        float qx;
        float qy;
        float qz;
        std::stringstream string_stream(line);
        string_stream >> x >> y >> z >> qw >> qx >> qy >> qz;
        Eigen::Quaternionf rotation_quaternion;
        rotation_quaternion.w() = qw;
        rotation_quaternion.x() = qx;
        rotation_quaternion.y() = qy;
        rotation_quaternion.z() = qz;
        Eigen::Matrix3f rotation_matrix =
            rotation_quaternion.toRotationMatrix();
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        transformation_matrix(0, 0) = rotation_matrix(0, 0);
        transformation_matrix(0, 1) = rotation_matrix(0, 1);
        transformation_matrix(0, 2) = rotation_matrix(0, 2);
        transformation_matrix(1, 0) = rotation_matrix(1, 0);
        transformation_matrix(1, 1) = rotation_matrix(1, 1);
        transformation_matrix(1, 2) = rotation_matrix(1, 2);
        transformation_matrix(2, 0) = rotation_matrix(2, 0);
        transformation_matrix(2, 1) = rotation_matrix(2, 1);
        transformation_matrix(2, 2) = rotation_matrix(2, 2);
        transformation_matrix(0, 3) = x;
        transformation_matrix(1, 3) = y;
        transformation_matrix(2, 3) = z;
        pose_matrix_vector_.push_back(transformation_matrix);
      }
    }
    std::cout << "read pose file successful" << std::endl;
  } else {
    std::cout << "read pose file failed" << std::endl;
  }
  pose_file.close();

  float min_x = FLT_MAX;
  float max_x = -FLT_MAX;
  float min_y = FLT_MAX;
  float max_y = -FLT_MAX;
  float min_z = FLT_MAX;
  float max_z = -FLT_MAX;
  for (int i = 0; i < pose_matrix_vector_.size(); i++) {
    if (pose_matrix_vector_[i](0, 3) < min_x) {
      min_x = pose_matrix_vector_[i](0, 3);
    }
    if (pose_matrix_vector_[i](0, 3) > max_x) {
      max_x = pose_matrix_vector_[i](0, 3);
    }
    if (pose_matrix_vector_[i](1, 3) < min_y) {
      min_y = pose_matrix_vector_[i](1, 3);
    }
    if (pose_matrix_vector_[i](1, 3) > max_y) {
      max_y = pose_matrix_vector_[i](1, 3);
    }
    if (pose_matrix_vector_[i](2, 3) < min_z) {
      min_z = pose_matrix_vector_[i](2, 3);
    }
    if (pose_matrix_vector_[i](2, 3) > max_z) {
      max_z = pose_matrix_vector_[i](2, 3);
    }
  }
  float length = fabs(min_x) > fabs(max_x) ? fabs(min_x) : fabs(max_x);
  float width = fabs(min_y) > fabs(max_y) ? fabs(min_y) : fabs(max_y);
  float height = fabs(min_z) > fabs(max_z) ? fabs(min_z) : fabs(max_z);
  float length_width_max = length > width ? length : width;
  float length_width_height_max =
      length_width_max > height ? length_width_max : height;

  range = length_width_height_max + lidar_max_range_;
}

//平面特征可视化，发布ros话题
void ViewPlane(std::vector<std::shared_ptr<OctreePlane>> &octree_planes,
               ros::Publisher &pub_plane_point_cloud,
               ros::Publisher &pub_plane_normal,
               Eigen::Matrix4f &view_transformation_matrix) {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr plane_point_cloud_map(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  visualization_msgs::MarkerArray plane_normal_map_msg;

  for (int i = 0; i < octree_planes.size(); i++) {
    int r = octree_planes[i]->plane_data->r;
    int g = octree_planes[i]->plane_data->g;
    int b = octree_planes[i]->plane_data->b;

    visualization_msgs::Marker marker_msg;
    marker_msg.header.frame_id = "wuba_base";  // 设置参考系
    marker_msg.header.stamp = ros::Time::now();
    marker_msg.ns = "plane_normal_map";
    marker_msg.id = i;
    marker_msg.type =
        visualization_msgs::Marker::ARROW;  // 设置marker类型为箭头
    marker_msg.action = visualization_msgs::Marker::ADD;
    // 设置箭头的尺寸
    marker_msg.scale.x = 0.1;   // 柄的直径
    marker_msg.scale.y = 0.36;  // 末端直径
    marker_msg.scale.z = 0.2;   // 箭头长度（除柄之外）
    // 设置箭头的颜色
    marker_msg.color.r = r / 255.0;  // 红色
    marker_msg.color.g = g / 255.0;  // 绿色
    marker_msg.color.b = b / 255.0;  // 蓝色
    marker_msg.color.a = 1.0;
    // 定义箭头的起点和终点
    geometry_msgs::Point geometry_point_start_msg, geometry_point_end_msg;
    Eigen::Vector4f plane_normal_4f = Eigen::Vector4f(
        octree_planes[i]->plane_normal[0], octree_planes[i]->plane_normal[1],
        octree_planes[i]->plane_normal[2], 1.0);
    Eigen::Vector4f plane_center_4f = Eigen::Vector4f(
        octree_planes[i]->plane_center[0], octree_planes[i]->plane_center[1],
        octree_planes[i]->plane_center[2], 1.0);
    plane_normal_4f = view_transformation_matrix * plane_normal_4f;
    plane_center_4f = view_transformation_matrix * plane_center_4f;
    Eigen::Vector3f plane_normal_3f =
        Eigen::Vector3f(plane_normal_4f[0] - view_transformation_matrix(0, 3),
                        plane_normal_4f[1] - view_transformation_matrix(1, 3),
                        plane_normal_4f[2] - view_transformation_matrix(2, 3));
    Eigen::Vector3f plane_center_3f = Eigen::Vector3f(
        plane_center_4f[0], plane_center_4f[1], plane_center_4f[2]);

    geometry_point_start_msg.x = plane_center_3f[0];
    geometry_point_start_msg.y = plane_center_3f[1];
    geometry_point_start_msg.z = plane_center_3f[2];  // 起点
    geometry_point_end_msg.x = plane_center_3f[0] + plane_normal_3f[0];
    geometry_point_end_msg.y = plane_center_3f[1] + plane_normal_3f[1];
    geometry_point_end_msg.z = plane_center_3f[2] + plane_normal_3f[2];  // 终点
    marker_msg.points.push_back(geometry_point_start_msg);
    marker_msg.points.push_back(geometry_point_end_msg);
    plane_normal_map_msg.markers.push_back(marker_msg);

    for (int j = 0; j < octree_planes[i]->plane_data->octree_point_cloud.size();
         j++) {
      Eigen::Vector4f plane_point_4f = Eigen::Vector4f(
          octree_planes[i]->plane_data->octree_point_cloud[j].x,
          octree_planes[i]->plane_data->octree_point_cloud[j].y,
          octree_planes[i]->plane_data->octree_point_cloud[j].z, 1.0);
      plane_point_4f = view_transformation_matrix * plane_point_4f;
      pcl::PointXYZRGB point_xyzrgb;
      point_xyzrgb.x = plane_point_4f[0];
      point_xyzrgb.y = plane_point_4f[1];
      point_xyzrgb.z = plane_point_4f[2];
      point_xyzrgb.r = r;
      point_xyzrgb.g = g;
      point_xyzrgb.b = b;
      (*plane_point_cloud_map).push_back(point_xyzrgb);
    }
  }

  sensor_msgs::PointCloud2 plane_point_cloud_map_msg;
  plane_point_cloud_map_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*plane_point_cloud_map, plane_point_cloud_map_msg);
  plane_point_cloud_map_msg.header.frame_id = "wuba_base";
  pub_plane_point_cloud.publish(plane_point_cloud_map_msg);

  pub_plane_normal.publish(plane_normal_map_msg);
}

//计算两个平面的角度（法向量）距离
float CalculatePlaneAngel(Eigen::Vector3f plane_normal_i,
                          Eigen::Vector3f plane_normal_j) {
  float plane_normal_i_dot_plane_normal_j = plane_normal_i.dot(plane_normal_j);
  float cos_theta = plane_normal_i_dot_plane_normal_j /
                    ((plane_normal_i.norm()) * (plane_normal_j.norm()));
  float theta = 180 * acos(cos_theta) / M_PI;
  return theta;
}

//计算两个平面的位置距离
float CalculatePlaneDistance(Eigen::Vector3f plane_center_i,
                             Eigen::Vector3f plane_center_j,
                             Eigen::Vector3f plane_normal_j) {
  float plane_a = plane_normal_j[0];
  float plane_b = plane_normal_j[1];
  float plane_c = plane_normal_j[2];
  float plane_d = -(plane_a * plane_center_j[0] + plane_b * plane_center_j[1] +
                    plane_c * plane_center_j[2]);
  float distance =
      fabs(plane_a * plane_center_i[0] + plane_b * plane_center_i[1] +
           plane_c * plane_center_i[2] + plane_d) /
      sqrt(pow(plane_a, 2) + pow(plane_b, 2) + pow(plane_c, 2));
  return distance;
}

//筛选地图的平面特征，减少平面数量，从而减少计算量
void SelectMapPlane(
    std::vector<std::shared_ptr<OctreePlane>> &octree_planes,
    std::vector<std::shared_ptr<OctreePlane>> &octree_planes_select,
    std::vector<std::vector<int>> &octree_planes_select_index,
    std::vector<PlaneSelectRule> &plane_select_rules) {
  octree_planes_select_index = std::vector<std::vector<int>>(
      plane_select_rules.size(), std::vector<int>());
  //按平面的大小进行排序
  for (int i = 0; i < octree_planes.size(); i++) {
    for (int j = 0; j < octree_planes.size(); j++) {
      if (j > i) {
        if (octree_planes[i]->plane_area < octree_planes[j]->plane_area) {
          std::shared_ptr<OctreePlane> octree_plane = octree_planes[i];
          octree_planes[i] = octree_planes[j];
          octree_planes[j] = octree_plane;
        }
      }
    }
  }
  for (int i = 0; i < octree_planes.size(); i++) {
    (octree_planes[i]->plane_normal).normalize();
    Eigen::Vector3f np_normal = Eigen::Vector3f(0.0, 0.0, 1.0);
    Eigen::Vector3f plane_normal = octree_planes[i]->plane_normal;
    float plane_angel = CalculatePlaneAngel(np_normal, plane_normal);
    for (int j = 0; j < plane_select_rules.size(); j++) {
      if (octree_planes[i]->plane_area >= plane_select_rules[j].min_area &&
          plane_angel >= plane_select_rules[j].min_angle &&
          plane_angel <= plane_select_rules[j].max_angle) {
        if (octree_planes_select_index[j].size() <=
            plane_select_rules[j].max_number) {
          octree_planes_select_index[j].push_back(octree_planes_select.size());
          octree_planes_select.push_back(octree_planes[i]);
          break;
        }
      }
    }
  }
  std::cout << "map plane_size: " << octree_planes_select.size() << std::endl;
  for (int i = 0; i < octree_planes_select_index.size(); i++) {
    std::cout << "---------------------------" << std::endl;
    for (int j = 0; j < octree_planes_select_index[i].size(); j++) {
      std::cout
          << octree_planes_select_index[i][j] << " -> "
          << octree_planes_select[octree_planes_select_index[i][j]]->plane_area
          << " "
          << octree_planes_select[octree_planes_select_index[i][j]]
                 ->plane_normal[0]
          << " "
          << octree_planes_select[octree_planes_select_index[i][j]]
                 ->plane_normal[1]
          << " "
          << octree_planes_select[octree_planes_select_index[i][j]]
                 ->plane_normal[2]
          << std::endl;
    }
  }
  std::cout << "---------------------------" << std::endl;
}

//筛选当前帧的平面特征，减少平面数量，从而减少计算量
void SelectScanPlane(
    std::vector<std::shared_ptr<OctreePlane>> &octree_planes,
    std::vector<std::shared_ptr<OctreePlane>> &octree_planes_select,
    std::vector<std::vector<int>> &octree_planes_select_index,
    std::vector<PlaneSelectRule> &plane_select_rules) {
  octree_planes_select_index = std::vector<std::vector<int>>(
      plane_select_rules.size(), std::vector<int>());
  //按平面的大小进行排序
  for (int i = 0; i < octree_planes.size(); i++) {
    for (int j = 0; j < octree_planes.size(); j++) {
      if (j > i) {
        if (octree_planes[i]->plane_area < octree_planes[j]->plane_area) {
          std::shared_ptr<OctreePlane> octree_plane = octree_planes[i];
          octree_planes[i] = octree_planes[j];
          octree_planes[j] = octree_plane;
        }
      }
    }
  }
  std::vector<std::vector<int>> octree_planes_select_index_temp(
      octree_planes_select_index.size(), std::vector<int>());
  for (int i = 0; i < octree_planes.size(); i++) {
    (octree_planes[i]->plane_normal).normalize();
    Eigen::Vector3f up_normal = Eigen::Vector3f(0.0, 0.0, 1.0);
    Eigen::Vector3f plane_normal = octree_planes[i]->plane_normal;
    float plane_angel = CalculatePlaneAngel(up_normal, plane_normal);
    for (int j = 0; j < plane_select_rules.size(); j++) {
      if (octree_planes[i]->plane_area >= plane_select_rules[j].min_area &&
          plane_angel >= plane_select_rules[j].min_angle &&
          plane_angel <= plane_select_rules[j].max_angle) {
        octree_planes_select_index_temp[j].push_back(i);
      }
    }
  }
  for (int i = 0; i < octree_planes_select_index_temp.size(); i++) {
    if (octree_planes_select_index_temp[i].size() <=
        plane_select_rules[i].max_number) {
      for (int j = 0; j < octree_planes_select_index_temp[i].size(); j++) {
        octree_planes_select_index[i].push_back(octree_planes_select.size());
        octree_planes_select.push_back(
            octree_planes[octree_planes_select_index_temp[i][j]]);
      }
    } else {
      //平面比较多的情况下，一个角度范围内只保留几个比较大的平面
      int angle_resolution = 360.0 / plane_select_rules[i].max_number;
      std::vector<std::vector<int>> octree_planes_angle_index(
          plane_select_rules[i].max_number, std::vector<int>());
      std::vector<std::vector<bool>> octree_planes_angle_index_selected(
          plane_select_rules[i].max_number, std::vector<bool>());
      for (int j = 0; j < octree_planes_select_index_temp[i].size(); j++) {
        float plane_angel =
            atan2(octree_planes[octree_planes_select_index_temp[i][j]]
                      ->plane_normal[1],
                  octree_planes[octree_planes_select_index_temp[i][j]]
                      ->plane_normal[0]);
        if (plane_angel < 0) {
          plane_angel = plane_angel + 2 * M_PI;
        }
        plane_angel = 180.0 * plane_angel / M_PI;
        int plane_index = plane_angel / angle_resolution;
        if (plane_index >= 0 &&
            plane_index < plane_select_rules[i].max_number) {
          octree_planes_angle_index[plane_index].push_back(
              octree_planes_select_index_temp[i][j]);
          octree_planes_angle_index_selected[plane_index].push_back(false);
        }
      }

      int select_depth = 0;
      while (octree_planes_select_index[i].size() <
             plane_select_rules[i].max_number) {
        int biggest_select = -1;
        int biggest_index = -1;
        int biggest_area = 0;
        for (int j = 0; j < octree_planes_angle_index.size(); j++) {
          if (octree_planes_angle_index[j].size() > select_depth) {
            if (octree_planes_angle_index_selected[j][select_depth] == false) {
              if (octree_planes[octree_planes_angle_index[j][select_depth]]
                      ->plane_area > biggest_area) {
                biggest_select = j;
                biggest_index = octree_planes_angle_index[j][select_depth];
                biggest_area =
                    octree_planes[octree_planes_angle_index[j][select_depth]]
                        ->plane_area;
              }
            }
          }
        }
        if (biggest_index >= 0) {
          octree_planes_select_index[i].push_back(octree_planes_select.size());
          octree_planes_select.push_back(octree_planes[biggest_index]);
          octree_planes_angle_index_selected[biggest_select][select_depth] =
              true;
        } else {
          select_depth++;
        }
      }
    }
  }
  std::cout << "scan plane_size: " << octree_planes_select.size() << std::endl;
  for (int i = 0; i < octree_planes_select_index.size(); i++) {
    std::cout << "---------------------------" << std::endl;
    for (int j = 0; j < octree_planes_select_index[i].size(); j++) {
      std::cout
          << octree_planes_select_index[i][j] << " -> "
          << octree_planes_select[octree_planes_select_index[i][j]]->plane_area
          << " "
          << octree_planes_select[octree_planes_select_index[i][j]]
                 ->plane_normal[0]
          << " "
          << octree_planes_select[octree_planes_select_index[i][j]]
                 ->plane_normal[1]
          << " "
          << octree_planes_select[octree_planes_select_index[i][j]]
                 ->plane_normal[2]
          << std::endl;
    }
  }
  std::cout << "---------------------------" << std::endl;
}

//三对对应平面计算变换矩阵
bool CalculatePlaneTransformation(Eigen::Vector3f &source_plane_center1,
                                  Eigen::Vector3f &source_plane_normal1,
                                  Eigen::Vector3f &source_plane_center2,
                                  Eigen::Vector3f &source_plane_normal2,
                                  Eigen::Vector3f &source_plane_center3,
                                  Eigen::Vector3f &source_plane_normal3,
                                  Eigen::Vector3f &target_plane_center1,
                                  Eigen::Vector3f &target_plane_normal1,
                                  Eigen::Vector3f &target_plane_center2,
                                  Eigen::Vector3f &target_plane_normal2,
                                  Eigen::Vector3f &target_plane_center3,
                                  Eigen::Vector3f &target_plane_normal3,
                                  Eigen::Matrix4f &transformation_matrix) {
  Eigen::Vector3f cs1 = source_plane_center1;
  Eigen::Vector3f ns1 = source_plane_normal1;
  Eigen::Vector3f cs2 = source_plane_center2;
  Eigen::Vector3f ns2 = source_plane_normal2;
  Eigen::Vector3f cs3 = source_plane_center3;
  Eigen::Vector3f ns3 = source_plane_normal3;
  Eigen::Vector3f ct1 = target_plane_center1;
  Eigen::Vector3f nt1 = target_plane_normal1;
  Eigen::Vector3f ct2 = target_plane_center2;
  Eigen::Vector3f nt2 = target_plane_normal2;
  Eigen::Vector3f ct3 = target_plane_center3;
  Eigen::Vector3f nt3 = target_plane_normal3;

  //计算旋转矩阵
  Eigen::Vector3f r1 = nt1.cross(ns1);
  r1.normalize();
  Eigen::Matrix3f r1x = Eigen::Matrix3f::Identity();
  r1x(0, 0) = 0;
  r1x(0, 1) = -r1[2];
  r1x(0, 2) = r1[1];
  r1x(1, 0) = r1[2];
  r1x(1, 1) = 0;
  r1x(1, 2) = -r1[0];
  r1x(2, 0) = -r1[1];
  r1x(2, 1) = r1[0];
  r1x(2, 2) = 0;
  Eigen::Matrix3f I = Eigen::Matrix3f::Identity();
  Eigen::Matrix3f R1 = Eigen::Matrix3f::Identity();
  float nt1dn1 = nt1.dot(ns1);
  Eigen::Vector3f r1cnt1 = r1.cross(nt1);
  float r1cnt1dns1 = r1cnt1.dot(ns1);
  float cos_theta1 = nt1dn1;
  float sin_theta1 = r1cnt1dns1;
  Eigen::Matrix3f rrt1 = r1 * (r1.transpose());
  R1 = cos_theta1 * I + (1 - cos_theta1) * rrt1 + sin_theta1 * r1x;

  nt2 = R1 * nt2;

  Eigen::Matrix3f R2 = Eigen::Matrix3f::Identity();
  Eigen::Vector3f r2 = ns1;
  Eigen::Matrix3f r2x = Eigen::Matrix3f::Identity();
  r2x(0, 0) = 0;
  r2x(0, 1) = -r2[2];
  r2x(0, 2) = r2[1];
  r2x(1, 0) = r2[2];
  r2x(1, 1) = 0;
  r2x(1, 2) = -r2[0];
  r2x(2, 0) = -r2[1];
  r2x(2, 1) = r2[0];
  r2x(2, 2) = 0;
  Eigen::Matrix3f rrt2 = r2 * (r2.transpose());
  float nt2dns2 = nt2.dot(ns2);
  float nt2dr2 = nt2.dot(r2);
  float ns2dr2 = ns2.dot(r2);
  Eigen::Vector3f r2cnt2 = r2.cross(nt2);
  float r2cnt2dns2 = r2cnt2.dot(ns2);
  float cos_theta2 = (nt2dns2 - (nt2dr2 * ns2dr2)) / (1 - (nt2dr2 * ns2dr2));
  float sin_theta2 = (r2cnt2dns2) / (1 - (nt2dr2 * ns2dr2));
  R2 = cos_theta2 * I + (1 - cos_theta2) * rrt2 + sin_theta2 * r2x;

  Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
  rotation_matrix = R2 * R1;
  transformation_matrix(0, 0) = rotation_matrix(0, 0);
  transformation_matrix(0, 1) = rotation_matrix(0, 1);
  transformation_matrix(0, 2) = rotation_matrix(0, 2);
  transformation_matrix(1, 0) = rotation_matrix(1, 0);
  transformation_matrix(1, 1) = rotation_matrix(1, 1);
  transformation_matrix(1, 2) = rotation_matrix(1, 2);
  transformation_matrix(2, 0) = rotation_matrix(2, 0);
  transformation_matrix(2, 1) = rotation_matrix(2, 1);
  transformation_matrix(2, 2) = rotation_matrix(2, 2);

  //恢复被旋转的量
  nt2 = target_plane_normal2;

  //将旋转矩阵作用到向量上
  ct1 = rotation_matrix * ct1;
  nt1 = rotation_matrix * nt1;
  ct2 = rotation_matrix * ct2;
  nt2 = rotation_matrix * nt2;
  ct3 = rotation_matrix * ct3;
  nt3 = rotation_matrix * nt3;

  //计算平移向量
  float ds1 = cs1.dot(ns1);
  float ds2 = cs2.dot(ns2);
  float ds3 = cs3.dot(ns3);
  float dt1 = ct1.dot(nt1);
  float dt2 = ct2.dot(nt2);
  float dt3 = ct3.dot(nt3);
  Eigen::Vector3f D = Eigen::Vector3f(ds1 - dt1, ds2 - dt2, ds3 - dt3);
  Eigen::Matrix3f A = Eigen::Matrix3f::Identity();
  A(0, 0) = ns1[0];
  A(0, 1) = ns1[1];
  A(0, 2) = ns1[2];
  A(1, 0) = ns2[0];
  A(1, 1) = ns2[1];
  A(1, 2) = ns2[2];
  A(2, 0) = ns3[0];
  A(2, 1) = ns3[1];
  A(2, 2) = ns3[2];
  Eigen::Matrix3f AT = A.transpose();
  Eigen::Vector3f T = ((AT * A).inverse()) * AT * D;
  transformation_matrix(0, 3) = T[0];
  transformation_matrix(1, 3) = T[1];
  transformation_matrix(2, 3) = T[2];

  if (std::isnan(transformation_matrix(0, 0)) ||
      std::isnan(transformation_matrix(0, 1)) ||
      std::isnan(transformation_matrix(0, 2)) ||
      std::isnan(transformation_matrix(0, 3)) ||
      std::isnan(transformation_matrix(1, 0)) ||
      std::isnan(transformation_matrix(1, 1)) ||
      std::isnan(transformation_matrix(1, 2)) ||
      std::isnan(transformation_matrix(1, 3)) ||
      std::isnan(transformation_matrix(2, 0)) ||
      std::isnan(transformation_matrix(2, 1)) ||
      std::isnan(transformation_matrix(2, 2)) ||
      std::isnan(transformation_matrix(2, 3))) {
    transformation_matrix = Eigen::Matrix4f::Identity();
    return false;
  } else {
    return true;
  }
}

//四元数转欧拉角
Eigen::Vector3f Quaternion2Euler(const Eigen::Quaternionf &q) {
  Eigen::Vector3f euler;
  double sinr_cosp = 2 * (q.w() * q.x() + q.y() * q.z());
  double cosr_cosp = 1 - 2 * (q.x() * q.x() + q.y() * q.y());
  euler(0) = std::atan2(sinr_cosp, cosr_cosp);

  double sinp = 2 * (q.w() * q.y() - q.z() * q.x());
  if (std::abs(sinp) >= 1)
    euler(1) = std::copysign(M_PI / 2, sinp);  // use 90 degrees if out of range
  else
    euler(1) = std::asin(sinp);

  double siny_cosp = 2 * (q.w() * q.z() + q.x() * q.y());
  double cosy_cosp = 1 - 2 * (q.y() * q.y() + q.z() * q.z());
  euler(2) = std::atan2(siny_cosp, cosy_cosp);
  return euler;

  // 第二种方法
  // Eigen::Vector3f euler =
  //     q.toRotationMatrix().eulerAngles(2, 1, 0); // ZYX顺序
}

//欧拉角和平移向量转变化矩阵
void EulerTranslation2Matrix(float roll, float pitch, float yaw, float tx,
                             float ty, float tz,
                             Eigen::Matrix4f &transformation_matrix) {
  Eigen::AngleAxisf rollAngle(roll, Eigen::Vector3f::UnitX());
  Eigen::AngleAxisf pitchAngle(pitch, Eigen::Vector3f::UnitY());
  Eigen::AngleAxisf yawAngle(yaw, Eigen::Vector3f::UnitZ());

  Eigen::Matrix3f rotation =
      (yawAngle * pitchAngle * rollAngle).toRotationMatrix();

  transformation_matrix(0, 0) = rotation(0, 0);
  transformation_matrix(0, 1) = rotation(0, 1);
  transformation_matrix(0, 2) = rotation(0, 2);
  transformation_matrix(0, 3) = tx;
  transformation_matrix(1, 0) = rotation(1, 0);
  transformation_matrix(1, 1) = rotation(1, 1);
  transformation_matrix(1, 2) = rotation(1, 2);
  transformation_matrix(1, 3) = ty;
  transformation_matrix(2, 0) = rotation(2, 0);
  transformation_matrix(2, 1) = rotation(2, 1);
  transformation_matrix(2, 2) = rotation(2, 2);
  transformation_matrix(2, 3) = tz;
}

//变换矩阵的聚类，合并相似的变化矩阵，减少需要验证的变化矩阵的数量
void TransformationCluster(
    std::vector<Eigen::Matrix4f> &transformation_matrix_vector) {
  clock_t start_cluster_transformation = clock();

  //存储变化矩阵的平移向量
  pcl::PointCloud<pcl::PointXYZ>::Ptr translation_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  //存储变化矩阵的欧拉角（变化矩阵的旋转）
  pcl::PointCloud<pcl::PointXYZ>::Ptr rotation_points(
      new pcl::PointCloud<pcl::PointXYZ>);
  for (int i = 0; i < transformation_matrix_vector.size(); i++) {
    pcl::PointXYZ point_translation;
    point_translation.x = transformation_matrix_vector[i](0, 3);
    point_translation.y = transformation_matrix_vector[i](1, 3);
    point_translation.z = transformation_matrix_vector[i](2, 3);
    translation_points->points.push_back(point_translation);

    Eigen::Matrix3f rotation_matrix = Eigen::Matrix3f::Identity();
    rotation_matrix(0, 0) = transformation_matrix_vector[i](0, 0);
    rotation_matrix(0, 1) = transformation_matrix_vector[i](0, 1);
    rotation_matrix(0, 2) = transformation_matrix_vector[i](0, 2);
    rotation_matrix(1, 0) = transformation_matrix_vector[i](1, 0);
    rotation_matrix(1, 1) = transformation_matrix_vector[i](1, 1);
    rotation_matrix(1, 2) = transformation_matrix_vector[i](1, 2);
    rotation_matrix(2, 0) = transformation_matrix_vector[i](2, 0);
    rotation_matrix(2, 1) = transformation_matrix_vector[i](2, 1);
    rotation_matrix(2, 2) = transformation_matrix_vector[i](2, 2);
    Eigen::Quaternionf quaternion(rotation_matrix);  // 旋转矩阵转四元数
    Eigen::Vector3f euler = Quaternion2Euler(quaternion);
    pcl::PointXYZ point_rotation;
    point_rotation.x = euler[0];
    point_rotation.y = euler[1];
    point_rotation.z = euler[2];
    rotation_points->points.push_back(point_rotation);
  }

  std::vector<Eigen::Matrix4f> cluster_transformation_matrix_vector;

  //先对变化矩阵的平移向量进行聚类
  pcl::KdTreeFLANN<pcl::PointXYZ> translation_kdtree;
  translation_kdtree.setInputCloud(translation_points);
  std::vector<bool> translation_visit_vector((*translation_points).size(),
                                             false);
  std::vector<std::vector<int>> translation_cluster;

  for (int i = 0; i < (*translation_points).size(); i++) {
    if (translation_visit_vector[i] == false) {
      translation_visit_vector[i] = true;
      std::vector<int> translation_add_vector;
      translation_add_vector.push_back(i);
      std::vector<int> translation_point_index;
      std::vector<float> translation_point_distance;
      if (translation_kdtree.radiusSearch(
              (*translation_points)[i], translation_radius_,
              translation_point_index, translation_point_distance) > 0) {
        for (int j = 0; j < translation_point_index.size(); j++) {
          if (translation_visit_vector[translation_point_index[j]] == false) {
            translation_add_vector.push_back(translation_point_index[j]);
            translation_visit_vector[translation_point_index[j]] = true;
          }
        }
      }
      translation_cluster.push_back(translation_add_vector);
    }
  }

  //对于每一个平移向量的聚类，再根据其欧拉角进行再次聚类后，计算平移向量和欧拉角的平均值，得到平均变化矩阵
  for (int c = 0; c < translation_cluster.size(); c++) {
    if (translation_cluster[c].size() == 1) {
      pcl::PointXYZ point_rotation =
          rotation_points->points[translation_cluster[c][0]];
      pcl::PointXYZ point_translation =
          translation_points->points[translation_cluster[c][0]];
      Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
      EulerTranslation2Matrix(point_rotation.x, point_rotation.y,
                              point_rotation.z, point_translation.x,
                              point_translation.y, point_translation.z,
                              transformation_matrix);
      cluster_transformation_matrix_vector.push_back(transformation_matrix);
    } else if (translation_cluster[c].size() == 2) {
      pcl::PointXYZ point_rotation_one =
          rotation_points->points[translation_cluster[c][0]];
      pcl::PointXYZ point_translation_one =
          translation_points->points[translation_cluster[c][0]];
      pcl::PointXYZ point_rotation_two =
          rotation_points->points[translation_cluster[c][1]];
      pcl::PointXYZ point_translation_two =
          translation_points->points[translation_cluster[c][1]];
      float distance_one_two =
          sqrt(pow(point_rotation_one.x - point_rotation_two.x, 2) +
               pow(point_rotation_one.y - point_rotation_two.y, 2) +
               pow(point_rotation_one.z - point_rotation_two.z, 2));
      if (distance_one_two < rotation_radius_) {
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        EulerTranslation2Matrix(
            (point_rotation_one.x + point_rotation_two.x) / 2.0,
            (point_rotation_one.y + point_rotation_two.y) / 2.0,
            (point_rotation_one.z + point_rotation_two.z) / 2.0,
            (point_translation_one.x + point_translation_two.x) / 2.0,
            (point_translation_one.y + point_translation_two.y) / 2.0,
            (point_translation_one.z + point_translation_two.z) / 2.0,
            transformation_matrix);
        cluster_transformation_matrix_vector.push_back(transformation_matrix);
      } else {
        Eigen::Matrix4f transformation_matrix_one = Eigen::Matrix4f::Identity();
        EulerTranslation2Matrix(
            point_rotation_one.x, point_rotation_one.y, point_rotation_one.z,
            point_translation_one.x, point_translation_one.y,
            point_translation_one.z, transformation_matrix_one);
        cluster_transformation_matrix_vector.push_back(
            transformation_matrix_one);
        Eigen::Matrix4f transformation_matrix_two = Eigen::Matrix4f::Identity();
        EulerTranslation2Matrix(
            point_rotation_two.x, point_rotation_two.y, point_rotation_two.z,
            point_translation_two.x, point_translation_two.y,
            point_translation_two.z, transformation_matrix_two);
        cluster_transformation_matrix_vector.push_back(
            transformation_matrix_two);
      }
    } else {
      std::vector<int> rotation_points_part_index;
      pcl::PointCloud<pcl::PointXYZ>::Ptr rotation_points_part(
          new pcl::PointCloud<pcl::PointXYZ>);
      for (int i = 0; i < translation_cluster[c].size(); i++) {
        (*rotation_points_part)
            .push_back((*rotation_points)[translation_cluster[c][i]]);
        rotation_points_part_index.push_back(translation_cluster[c][i]);
      }

      pcl::KdTreeFLANN<pcl::PointXYZ> rotation_kdtree;
      rotation_kdtree.setInputCloud(rotation_points_part);
      std::vector<bool> rotation_visit_vector((*rotation_points_part).size(),
                                              false);
      std::vector<std::vector<int>> rotation_cluster;

      for (int i = 0; i < (*rotation_points_part).size(); i++) {
        if (rotation_visit_vector[i] == false) {
          rotation_visit_vector[i] = true;
          std::vector<int> rotation_add_vector;
          rotation_add_vector.push_back(i);
          std::vector<int> rotation_point_index;
          std::vector<float> rotation_point_distance;
          if (rotation_kdtree.radiusSearch(
                  (*rotation_points_part)[i], rotation_radius_,
                  rotation_point_index, rotation_point_distance) > 0) {
            for (int j = 0; j < rotation_point_index.size(); j++) {
              if (rotation_visit_vector[rotation_point_index[j]] == false) {
                rotation_add_vector.push_back(rotation_point_index[j]);
                rotation_visit_vector[rotation_point_index[j]] = true;
              }
            }
          }
          rotation_cluster.push_back(rotation_add_vector);
        }
      }

      for (int i = 0; i < rotation_cluster.size(); i++) {
        float average_x = 0;
        float average_y = 0;
        float average_z = 0;
        float average_roll = 0;
        float average_pitch = 0;
        float average_yaw = 0;
        for (int j = 0; j < rotation_cluster[i].size(); j++) {
          int index = rotation_points_part_index[rotation_cluster[i][j]];
          average_x += translation_points->points[index].x;
          average_y += translation_points->points[index].y;
          average_z += translation_points->points[index].z;
          average_roll += rotation_points->points[index].x;
          average_pitch += rotation_points->points[index].y;
          average_yaw += rotation_points->points[index].z;
        }
        average_x = average_x / rotation_cluster[i].size();
        average_y = average_y / rotation_cluster[i].size();
        average_z = average_z / rotation_cluster[i].size();
        average_roll = average_roll / rotation_cluster[i].size();
        average_pitch = average_pitch / rotation_cluster[i].size();
        average_yaw = average_yaw / rotation_cluster[i].size();
        Eigen::Matrix4f transformation_matrix = Eigen::Matrix4f::Identity();
        EulerTranslation2Matrix(average_roll, average_pitch, average_yaw,
                                average_x, average_y, average_z,
                                transformation_matrix);
        cluster_transformation_matrix_vector.push_back(transformation_matrix);
      }
    }
  }

  std::cout << "cluster_transformation_matrix_vector size: "
            << cluster_transformation_matrix_vector.size() << std::endl;
  cluster_transformation_matrix_vector.swap(transformation_matrix_vector);

  clock_t end_cluster_transformation = clock();
  double cluster_transformation_time =
      ((double)(end_cluster_transformation - start_cluster_transformation) /
       (double)CLOCKS_PER_SEC);
  std::cout << "cluster_transformation_time: " << cluster_transformation_time
            << std::endl;

  std::cout << "---------------------------" << std::endl;
}

void PlaneLocalization(
    std::vector<std::shared_ptr<OctreePlane>> &octree_planes_scan,
    std::vector<std::shared_ptr<OctreePlane>> &octree_planes_scan_select,
    std::vector<std::vector<int>> &octree_planes_scan_select_index,
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
    Eigen::Matrix4f &best_transformation_matrix) {
  clock_t start_localization = clock();

  //遍历所有三对对应平面，计算候选变换矩阵
  std::vector<Eigen::Matrix4f> transformation_matrix_vector;
  for (int i = 0; i < octree_planes_scan_select_index.size() - 1; i++) {
    for (int j = 0; j < octree_planes_scan_select_index[i].size(); j++) {
      Eigen::Vector3f frist_scan_plane_normal =
          octree_planes_scan_select[octree_planes_scan_select_index[i][j]]
              ->plane_normal;
      Eigen::Vector3f frist_scan_plane_center =
          octree_planes_scan_select[octree_planes_scan_select_index[i][j]]
              ->plane_center;
      for (int k = 0; k < octree_planes_map_select_index_.size() - 1; k++) {
        for (int l = 0; l < octree_planes_map_select_index_[k].size(); l++) {
          Eigen::Vector3f frist_map_plane_normal =
              octree_planes_map_select_[octree_planes_map_select_index_[k][l]]
                  ->plane_normal;
          Eigen::Vector3f frist_map_plane_center =
              octree_planes_map_select_[octree_planes_map_select_index_[k][l]]
                  ->plane_center;
          for (int m = i + 1; m < octree_planes_scan_select_index.size(); m++) {
            for (int n = 0; n < octree_planes_scan_select_index[m].size();
                 n++) {
              Eigen::Vector3f second_scan_plane_normal =
                  octree_planes_scan_select[octree_planes_scan_select_index[m]
                                                                           [n]]
                      ->plane_normal;
              Eigen::Vector3f second_scan_plane_center =
                  octree_planes_scan_select[octree_planes_scan_select_index[m]
                                                                           [n]]
                      ->plane_center;
              for (int o = k + 1; o < octree_planes_map_select_index_.size();
                   o++) {
                for (int p = 0; p < octree_planes_map_select_index_[o].size();
                     p++) {
                  Eigen::Vector3f second_map_plane_normal =
                      octree_planes_map_select_
                          [octree_planes_map_select_index_[o][p]]
                              ->plane_normal;
                  Eigen::Vector3f second_map_plane_center =
                      octree_planes_map_select_
                          [octree_planes_map_select_index_[o][p]]
                              ->plane_center;
                  float frist_second_scan_plane_angle = CalculatePlaneAngel(
                      frist_scan_plane_normal, second_scan_plane_normal);
                  float frist_second_map_plane_angle = CalculatePlaneAngel(
                      frist_map_plane_normal, second_map_plane_normal);
                  //选取的scan和map第一和第二两对平面的角度要相近
                  if (fabs(frist_second_scan_plane_angle -
                           frist_second_map_plane_angle) <
                      select_plane_angel_threshold_) {
                    for (int q = 0;
                         q < octree_planes_scan_select_index[m].size(); q++) {
                      if (n != q) {
                        Eigen::Vector3f third_scan_plane_normal =
                            octree_planes_scan_select
                                [octree_planes_scan_select_index[m][q]]
                                    ->plane_normal;
                        Eigen::Vector3f third_scan_plane_center =
                            octree_planes_scan_select
                                [octree_planes_scan_select_index[m][q]]
                                    ->plane_center;
                        float second_third_scan_plane_angle =
                            CalculatePlaneAngel(second_scan_plane_normal,
                                                third_scan_plane_normal);
                        //选取的scan的第二和第三个平面的角度要比较大
                        if (second_third_scan_plane_angle >
                                second_third_plane_angel_threshold_ &&
                            second_third_scan_plane_angle <
                                (180.0 - second_third_plane_angel_threshold_)) {
                          float first_third_scan_plane_angle =
                              CalculatePlaneAngel(frist_scan_plane_normal,
                                                  third_scan_plane_normal);
                          for (int r = 0;
                               r < octree_planes_map_select_index_[o].size();
                               r++) {
                            if (p != r) {
                              Eigen::Vector3f third_map_plane_normal =
                                  octree_planes_map_select_
                                      [octree_planes_map_select_index_[o][r]]
                                          ->plane_normal;
                              Eigen::Vector3f third_map_plane_center =
                                  octree_planes_map_select_
                                      [octree_planes_map_select_index_[o][r]]
                                          ->plane_center;
                              float second_third_map_plane_angle =
                                  CalculatePlaneAngel(second_map_plane_normal,
                                                      third_map_plane_normal);
                              float first_third_map_plane_angle =
                                  CalculatePlaneAngel(frist_map_plane_normal,
                                                      third_map_plane_normal);
                              if (fabs(second_third_scan_plane_angle -
                                       second_third_map_plane_angle) <
                                      select_plane_angel_threshold_ &&
                                  fabs(first_third_scan_plane_angle -
                                       first_third_map_plane_angle) <
                                      select_plane_angel_threshold_) {
                                Eigen::Matrix4f transformation_matrix =
                                    Eigen::Matrix4f::Identity();
                                //根据三对对应平面计算候选变换矩阵
                                bool get_transformation =
                                    CalculatePlaneTransformation(
                                        frist_map_plane_center,
                                        frist_map_plane_normal,
                                        second_map_plane_center,
                                        second_map_plane_normal,
                                        third_map_plane_center,
                                        third_map_plane_normal,
                                        frist_scan_plane_center,
                                        frist_scan_plane_normal,
                                        second_scan_plane_center,
                                        second_scan_plane_normal,
                                        third_scan_plane_center,
                                        third_scan_plane_normal,
                                        transformation_matrix);
                                if (get_transformation == true) {
                                  Eigen::Vector3f query_point = Eigen::Vector3f(
                                      transformation_matrix(0, 3),
                                      transformation_matrix(1, 3),
                                      transformation_matrix(2, 3));
                                  //默认变化矩阵的平移向量应该在占用八叉树的地图内，如果不再判断此变换矩阵无效，不将其加入候选变换矩阵
                                  bool is_occupancy =
                                      plane_octree_occupy_map_->QueryOccupancy(
                                          query_point,
                                          plane_octree_occupy_map_depth_ - 1);
                                  if (is_occupancy == true) {
                                    transformation_matrix_vector.push_back(
                                        transformation_matrix);
                                  }
                                }
                              }
                            }
                          }
                        }
                      }
                    }
                  }
                }
              }
            }
          }
        }
      }
    }
  }

  std::cout << "transformation_matrix_vector size: "
            << transformation_matrix_vector.size() << std::endl;

  //变换矩阵聚类，减少需要验证的变换矩阵数量
  TransformationCluster(transformation_matrix_vector);

  //若没有候选变换矩阵则重定位失败
  if (transformation_matrix_vector.size() == 0) {
    std::cout << "localization failed" << std::endl;
    return;
  }

  //候选变化矩阵可视化，发布ros话题
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformation_matrix_points_all(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  for (int i = 0; i < transformation_matrix_vector.size(); i++) {
    int r = rand() % 255;
    int g = rand() % 255;
    int b = rand() % 255;
    pcl::PointXYZRGB point_xyzrgb;
    point_xyzrgb.r = r;
    point_xyzrgb.g = g;
    point_xyzrgb.b = b;
    point_xyzrgb.x = transformation_matrix_vector[i](0, 3);
    point_xyzrgb.y = transformation_matrix_vector[i](1, 3);
    point_xyzrgb.z = transformation_matrix_vector[i](2, 3);
    transformation_matrix_points_all->points.push_back(point_xyzrgb);
  }
  sensor_msgs::PointCloud2 transformation_matrix_points_all_msg;
  transformation_matrix_points_all_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*transformation_matrix_points_all,
                transformation_matrix_points_all_msg);
  transformation_matrix_points_all_msg.header.frame_id = "wuba_base";
  pub_transformation_matrix_points_all_.publish(
      transformation_matrix_points_all_msg);

  //初步验证，基于平面特征计算变换矩阵的得分，对所有的候选变换矩阵进行筛选排序
  std::vector<ScoreIndex> transformation_score_index_vector;
  for (int i = 0; i < transformation_matrix_vector.size(); i++) {
    float transformation_score = 0;
    for (int j = 0; j < octree_planes_scan_select.size(); j++) {
      Eigen::Vector4f scan_plane_normal_4f =
          Eigen::Vector4f(octree_planes_scan_select[j]->plane_normal[0],
                          octree_planes_scan_select[j]->plane_normal[1],
                          octree_planes_scan_select[j]->plane_normal[2], 1.0);
      Eigen::Vector4f scan_plane_center_4f =
          Eigen::Vector4f(octree_planes_scan_select[j]->plane_center[0],
                          octree_planes_scan_select[j]->plane_center[1],
                          octree_planes_scan_select[j]->plane_center[2], 1.0);
      scan_plane_normal_4f =
          transformation_matrix_vector[i] * scan_plane_normal_4f;
      scan_plane_center_4f =
          transformation_matrix_vector[i] * scan_plane_center_4f;
      Eigen::Vector3f scan_plane_normal_3f = Eigen::Vector3f(
          scan_plane_normal_4f[0] - transformation_matrix_vector[i](0, 3),
          scan_plane_normal_4f[1] - transformation_matrix_vector[i](1, 3),
          scan_plane_normal_4f[2] - transformation_matrix_vector[i](2, 3));
      Eigen::Vector3f scan_plane_center_3f =
          Eigen::Vector3f(scan_plane_center_4f[0], scan_plane_center_4f[1],
                          scan_plane_center_4f[2]);
      int scan_plane_area = octree_planes_scan_select[j]->plane_area;
      float best_coplanar_score = 0;
      for (int k = 0; k < octree_planes_map_select_.size(); k++) {
        Eigen::Vector3f map_plane_normal_3f =
            octree_planes_map_select_[k]->plane_normal;
        Eigen::Vector3f map_plane_center_3f =
            octree_planes_map_select_[k]->plane_center;
        float scan_map_angle =
            CalculatePlaneAngel(scan_plane_normal_3f, map_plane_normal_3f);
        float scan_map_distance_a = CalculatePlaneDistance(
            scan_plane_center_3f, map_plane_center_3f, map_plane_normal_3f);
        float scan_map_distance_b = CalculatePlaneDistance(
            map_plane_center_3f, scan_plane_center_3f, scan_plane_normal_3f);
        float scan_map_distance = scan_map_distance_a > scan_map_distance_b
                                      ? scan_map_distance_a
                                      : scan_map_distance_b;
        float scan_map_plane_error =
            scan_map_angle * coplanar_angel_weight_ +
            scan_map_distance * coplanar_distance_weight_;
        int map_plane_area = octree_planes_map_select_[j]->plane_area;
        float coplanar_score =
            scan_plane_area < map_plane_area ? scan_plane_area : map_plane_area;
        coplanar_score = std::exp(-coplanar_attenuation_coefficient_ *
                                  scan_map_plane_error) *
                         coplanar_score;
        if (coplanar_score > best_coplanar_score) {
          best_coplanar_score = coplanar_score;
        }
      }
      transformation_score += best_coplanar_score;
    }
    ScoreIndex score_index;
    score_index.score = transformation_score;
    score_index.index = i;
    transformation_score_index_vector.push_back(score_index);
  }

  //按初步验证的得分对变化候选变换矩阵进行排序
  for (int i = 0; i < transformation_score_index_vector.size(); i++) {
    for (int j = 0; j < transformation_score_index_vector.size(); j++) {
      if (j > i) {
        if (transformation_score_index_vector[i].score <
            transformation_score_index_vector[j].score) {
          ScoreIndex score_index_temp = transformation_score_index_vector[i];
          transformation_score_index_vector[i] =
              transformation_score_index_vector[j];
          transformation_score_index_vector[j] = score_index_temp;
        }
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformation_matrix_points_good(
      new pcl::PointCloud<pcl::PointXYZRGB>);

  clock_t start_fine_verify_octree = clock();

  //精细验证，基于占用八叉树地图计算变换矩阵得分，选取最佳变换矩阵
  int best_transformation_index = -1;
  int best_transformation_score = 0;
  verify_number_ = verify_number_ > (transformation_matrix_vector.size() - 1)
                       ? (transformation_matrix_vector.size() - 1)
                       : verify_number_;
  for (int i = 0; i < verify_number_; i++) {
    int r = rand() % 255;
    int g = rand() % 255;
    int b = rand() % 255;
    pcl::PointXYZRGB point_xyzrgb;
    point_xyzrgb.r = r;
    point_xyzrgb.g = g;
    point_xyzrgb.b = b;
    point_xyzrgb.x =
        transformation_matrix_vector[transformation_score_index_vector[i]
                                         .index](0, 3);
    point_xyzrgb.y =
        transformation_matrix_vector[transformation_score_index_vector[i]
                                         .index](1, 3);
    point_xyzrgb.z =
        transformation_matrix_vector[transformation_score_index_vector[i]
                                         .index](2, 3);
    transformation_matrix_points_good->points.push_back(point_xyzrgb);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(
        *point_cloud, *point_cloud_transformed,
        transformation_matrix_vector[transformation_score_index_vector[i]
                                         .index]);
    int transformation_score = plane_octree_occupy_map_->QueryOccupancys(
        point_cloud_transformed, plane_octree_occupy_map_depth_ + 1);
    if (transformation_score > best_transformation_score) {
      best_transformation_index = transformation_score_index_vector[i].index;
      best_transformation_score = transformation_score;
    }
  }

  //打印耗时
  clock_t end_fine_verify_octree = clock();
  double fine_verify_octree_time =
      ((double)(end_fine_verify_octree - start_fine_verify_octree) /
       (double)CLOCKS_PER_SEC);
  std::cout << "fine_verify_octree_time: " << fine_verify_octree_time
            << std::endl;

  if (best_transformation_index >= 0) {
    std::cout << "best_transformation_score: " << best_transformation_score
              << " best_transformation_matrix: " << std::endl;
    std::cout << transformation_matrix_vector[best_transformation_index]
              << std::endl;
    best_transformation_matrix =
        transformation_matrix_vector[best_transformation_index];
  }

  //显示初步验证得分比较高的变化矩阵，发布ros话题
  sensor_msgs::PointCloud2 transformation_matrix_points_good_msg;
  transformation_matrix_points_good_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*transformation_matrix_points_good,
                transformation_matrix_points_good_msg);
  transformation_matrix_points_good_msg.header.frame_id = "wuba_base";
  pub_transformation_matrix_points_good_.publish(
      transformation_matrix_points_good_msg);

  clock_t end_localization = clock();
  double localization_time = ((double)(end_localization - start_localization) /
                              (double)CLOCKS_PER_SEC);
  std::cout << "localization_time: " << localization_time << std::endl;
}

//测试基于平面特征的重定位
void TestLocalization() {
  //构造scan八叉树
  PlaneOctree plane_octree_scan(40.0, 0.5, 5, 0.05, 0.4, 5.0, 0.2);
  std::string scan_name = current_path_ + "/src/plane_localization/data/pcd/" +
                          std::to_string(test_index_) + ".pcd";
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_name, *point_cloud) == -1) {
    PCL_ERROR("Couldn't read file \n");
    return;
  }
  plane_octree_scan.OctreePlaneAddPointCloud(point_cloud);
  std::vector<std::shared_ptr<OctreePlane>> octree_planes_scan;
  plane_octree_scan.OctreePlaneExtraction(octree_planes_scan, true);
  std::vector<std::shared_ptr<OctreePlane>> octree_planes_scan_select;
  std::vector<std::vector<int>> octree_planes_scan_select_index;
  SelectScanPlane(octree_planes_scan, octree_planes_scan_select,
                  octree_planes_scan_select_index, scan_plane_select_rules_);

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
  voxel_grid.setInputCloud(point_cloud);
  voxel_grid.setLeafSize(0.5, 0.5, 0.5);
  voxel_grid.filter(*point_cloud_filtered);

  Eigen::Matrix4f best_transformation_matrix = Eigen::Matrix4f::Identity();
  PlaneLocalization(octree_planes_scan, octree_planes_scan_select,
                    octree_planes_scan_select_index, point_cloud_filtered,
                    best_transformation_matrix);

  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
      new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*point_cloud_filtered, *point_cloud_transformed,
                           best_transformation_matrix);

  sensor_msgs::PointCloud2 point_cloud_scan_msg;
  point_cloud_scan_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*point_cloud_transformed, point_cloud_scan_msg);
  point_cloud_scan_msg.header.frame_id = "wuba_base";
  pub_scan_points_.publish(point_cloud_scan_msg);

  ViewPlane(octree_planes_scan_select, pub_plane_point_cloud_scan_,
            pub_plane_normal_scan_, best_transformation_matrix);
}

//重定位点前点云帧回调函数，只进行一次重定位
void LidarCallback(const livox_ros_driver::CustomMsgConstPtr msg) {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (unsigned int i = 0; i < msg->point_num; ++i) {
    pcl::PointXYZ point_xyz;
    point_xyz.x = msg->points[i].x;
    point_xyz.y = msg->points[i].y;
    point_xyz.z = msg->points[i].z;
    (*point_cloud).push_back(point_xyz);
  }

  if (get_test_data_ == false) {
    //构造scan八叉树
    PlaneOctree plane_octree_scan(40.0, 0.5, 5, 0.05, 0.4, 5.0, 0.2);
    plane_octree_scan.OctreePlaneAddPointCloud(point_cloud);
    std::vector<std::shared_ptr<OctreePlane>> octree_planes_scan;
    plane_octree_scan.OctreePlaneExtraction(octree_planes_scan, true);
    std::vector<std::shared_ptr<OctreePlane>> octree_planes_scan_select;
    std::vector<std::vector<int>> octree_planes_scan_select_index;
    SelectScanPlane(octree_planes_scan, octree_planes_scan_select,
                    octree_planes_scan_select_index, scan_plane_select_rules_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(point_cloud);
    voxel_grid.setLeafSize(0.5, 0.5, 0.5);
    voxel_grid.filter(*point_cloud_filtered);

    Eigen::Matrix4f best_transformation_matrix = Eigen::Matrix4f::Identity();
    PlaneLocalization(octree_planes_scan, octree_planes_scan_select,
                      octree_planes_scan_select_index, point_cloud_filtered,
                      best_transformation_matrix);
    ViewPlane(octree_planes_scan_select, pub_plane_point_cloud_scan_,
              pub_plane_normal_scan_, best_transformation_matrix);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*point_cloud_filtered, *point_cloud_transformed,
                             best_transformation_matrix);

    sensor_msgs::PointCloud2 point_cloud_scan_msg;
    point_cloud_scan_msg.header.stamp = ros::Time::now();
    pcl::toROSMsg(*point_cloud_transformed, point_cloud_scan_msg);
    point_cloud_scan_msg.header.frame_id = "wuba_base";
    pub_scan_points_.publish(point_cloud_scan_msg);

    get_test_data_ = true;
  }
}

//点云地图可视化
void ViewMapPoints() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_map(
      new pcl::PointCloud<pcl::PointXYZ>);

  for (int i = 0; i < pose_matrix_vector_.size(); i++) {
    std::string scan_name = current_path_ +
                            "/src/plane_localization/data/pcd/" +
                            std::to_string(i) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_name, *point_cloud) == -1) {
      PCL_ERROR("Couldn't read file \n");
      return;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_filtered(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setInputCloud(point_cloud);
    voxel_grid.setLeafSize(0.5, 0.5, 0.5);
    voxel_grid.filter(*point_cloud_filtered);

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*point_cloud_filtered, *point_cloud_transformed,
                             pose_matrix_vector_[i]);
    (*point_cloud_map) += (*point_cloud_transformed);
  }

  std::cout << "map size: " << (*point_cloud_map).size() << std::endl;

  sensor_msgs::PointCloud2 point_cloud_map_msg;
  point_cloud_map_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*point_cloud_map, point_cloud_map_msg);
  point_cloud_map_msg.header.frame_id = "wuba_base";
  pub_map_points_.publish(point_cloud_map_msg);
}

//定义ROS订阅和发布
void RegisterSubscribeAndPublish(ros::NodeHandle &nh) {
  //订阅点云话题，进行重定位
  sub_lidar_cloud_ = nh.subscribe<livox_ros_driver::CustomMsg>(
      "/livox/lidar", 100, LidarCallback);

  pub_plane_point_cloud_map_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/livox/plane_point_cloud_map", 1, true);
  pub_plane_normal_map_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/livox/plane_normal_map", 1, true);
  pub_plane_point_cloud_scan_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/livox/plane_point_cloud_scan", 1, true);
  pub_plane_normal_scan_ = nh.advertise<visualization_msgs::MarkerArray>(
      "/livox/plane_normal_scan", 1, true);

  pub_transformation_matrix_points_all_ =
      nh.advertise<sensor_msgs::PointCloud2>(
          "/livox/transformation_matrix_points_all", 1, true);
  pub_transformation_matrix_points_good_ =
      nh.advertise<sensor_msgs::PointCloud2>(
          "/livox/transformation_matrix_points_good", 1, true);
  pub_point_cloud_occupy_map_ = nh.advertise<sensor_msgs::PointCloud2>(
      "/livox/point_cloud_occupy", 1, true);

  pub_map_points_ =
      nh.advertise<sensor_msgs::PointCloud2>("/livox/map_points", 1, true);
  pub_scan_points_ =
      nh.advertise<sensor_msgs::PointCloud2>("/livox/scan_points", 1, true);
}

//使用八叉树构建地图
void ConstructOctreeMap(float map_range) {
  clock_t start_create_octree = clock();

  //八叉树占用地图，用于变化矩阵的验证。
  //构造函数的参数分别为:
  //八叉树地图的跨度、八叉树地图的分辨率、判断叶子节点为平面的最少点数量、判断叶子节点为平面的最小特征和最大特征值的比值、
  //判断叶子节点为平面的次大特征和最大特征值比值、平面融合时的法向量角度阈值、平面融合时一平面中心到另一平面的距离
  plane_octree_occupy_map_ =
      std::make_shared<PlaneOctree>(map_range, 0.5, 20, 0.05, 0.4, 8.0, 0.35);

  //对单帧的平面特征提取的八叉树
  PlaneOctree plane_octree_scan(40.0, 0.5, 6, 0.05, 0.4, 5.0, 0.2);
  //融合多帧平面特征的八叉树
  PlaneOctree plane_octree_map(map_range, 0.5, 20, 0.05, 0.4, 8.0, 0.35);
  for (int i = 0; i < pose_matrix_vector_.size(); i++) {
    std::string scan_name = current_path_ +
                            "/src/plane_localization/data/pcd/" +
                            std::to_string(i) + ".pcd";
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud(
        new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(scan_name, *point_cloud) == -1) {
      PCL_ERROR("Couldn't read file \n");
      return;
    }

    //构造八叉树占用地图
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud_transformed(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*point_cloud, *point_cloud_transformed,
                             pose_matrix_vector_[i]);
    plane_octree_occupy_map_->OctreePlaneAddPointCloud(point_cloud_transformed);

    //提取单帧的平面特征
    plane_octree_scan.ResetPlaneOctree();
    plane_octree_scan.OctreePlaneAddPointCloud(point_cloud);
    std::vector<std::shared_ptr<OctreePlane>> octree_planes_scan;
    plane_octree_scan.OctreePlaneExtraction(octree_planes_scan, true);
    //将单帧的平面特征转到地图坐标系中
    for (int j = 0; j < octree_planes_scan.size(); j++) {
      (octree_planes_scan[j]->plane_normal).normalize();
      Eigen::Vector4f plane_normal =
          Eigen::Vector4f(octree_planes_scan[j]->plane_normal[0],
                          octree_planes_scan[j]->plane_normal[1],
                          octree_planes_scan[j]->plane_normal[2], 1.0);
      Eigen::Vector4f plane_center =
          Eigen::Vector4f(octree_planes_scan[j]->plane_center[0],
                          octree_planes_scan[j]->plane_center[1],
                          octree_planes_scan[j]->plane_center[2], 1.0);
      plane_normal = pose_matrix_vector_[i] * plane_normal;
      plane_center = pose_matrix_vector_[i] * plane_center;
      octree_planes_scan[j]->plane_normal[0] =
          plane_normal[0] - pose_matrix_vector_[i](0, 3);
      octree_planes_scan[j]->plane_normal[1] =
          plane_normal[1] - pose_matrix_vector_[i](1, 3);
      octree_planes_scan[j]->plane_normal[2] =
          plane_normal[2] - pose_matrix_vector_[i](2, 3);
      octree_planes_scan[j]->plane_center[0] = plane_center[0];
      octree_planes_scan[j]->plane_center[1] = plane_center[1];
      octree_planes_scan[j]->plane_center[2] = plane_center[2];
      for (int k = 0;
           k < octree_planes_scan[j]->plane_data->octree_point_cloud.size();
           k++) {
        Eigen::Vector4f plane_point = Eigen::Vector4f(
            octree_planes_scan[j]->plane_data->octree_point_cloud[k].x,
            octree_planes_scan[j]->plane_data->octree_point_cloud[k].y,
            octree_planes_scan[j]->plane_data->octree_point_cloud[k].z, 1.0);
        plane_point = pose_matrix_vector_[i] * plane_point;
        octree_planes_scan[j]->plane_data->octree_point_cloud[k].x =
            plane_point[0];
        octree_planes_scan[j]->plane_data->octree_point_cloud[k].y =
            plane_point[1];
        octree_planes_scan[j]->plane_data->octree_point_cloud[k].z =
            plane_point[2];
      }
      for (int k = 0;
           k < octree_planes_scan[j]->patch_data->octree_point_cloud.size();
           k++) {
        Eigen::Vector4f plane_patch = Eigen::Vector4f(
            octree_planes_scan[j]->patch_data->octree_point_cloud[k].x,
            octree_planes_scan[j]->patch_data->octree_point_cloud[k].y,
            octree_planes_scan[j]->patch_data->octree_point_cloud[k].z, 1.0);
        plane_patch = pose_matrix_vector_[i] * plane_patch;
        octree_planes_scan[j]->patch_data->octree_point_cloud[k].x =
            plane_patch[0];
        octree_planes_scan[j]->patch_data->octree_point_cloud[k].y =
            plane_patch[1];
        octree_planes_scan[j]->patch_data->octree_point_cloud[k].z =
            plane_patch[2];
      }
    }
    //将平面特征加入到平面特征融合八叉树中
    plane_octree_map.OctreePlaneAddPlanes(octree_planes_scan);
  }
  //融合平面特征融合八叉树中的平面
  plane_octree_map.OctreePlaneExtraction(octree_planes_map_, false);

  //重新计算融合后的全局平面的面积
  plane_octree_map.GetOctreePlaneArea(octree_planes_map_);

  //按map_plane_select_rules_的设置选取平面特征
  SelectMapPlane(octree_planes_map_, octree_planes_map_select_,
                 octree_planes_map_select_index_, map_plane_select_rules_);

  //将八叉树占用地图转为二进制，存入硬盘
  std::string octree_occupy_binary_name =
      current_path_ + "/src/plane_localization/data/octree_occupy.bin";
  std::vector<char> octree_occupy_binary_out_buffer;
  plane_octree_occupy_map_->OctreeOccupyToBinary(
      plane_octree_occupy_map_->point_cloud_octree_data_,
      octree_occupy_binary_out_buffer);
  // 将缓冲区写入文件
  std::ofstream octree_occupy_binary_out(octree_occupy_binary_name,
                                         std::ios::binary);
  if (octree_occupy_binary_out) {
    // 直接写入vector的连续内存块
    octree_occupy_binary_out.write(
        reinterpret_cast<char *>(octree_occupy_binary_out_buffer.data()),
        static_cast<std::streamsize>(octree_occupy_binary_out_buffer.size()));

    // 检查是否写入成功
    if (!octree_occupy_binary_out.good()) {
      std::cerr << "unable to write file!" << std::endl;
      return;
    }
    octree_occupy_binary_out.close();
  } else {
    std::cerr << "unable to open file!" << std::endl;
    return;
  }

  //将地图平特征转为二进制，存入硬盘
  std::string octree_planes_binary_name =
      current_path_ + "/src/plane_localization/data/octree_planes.bin";
  std::vector<char> octree_planes_binary_out_buffer;
  plane_octree_map.OctreePlanesToBinary(octree_planes_map_,
                                        octree_planes_binary_out_buffer);
  // 将缓冲区写入文件
  std::ofstream octree_planes_binary_out(octree_planes_binary_name,
                                         std::ios::binary);
  if (octree_planes_binary_out) {
    // 直接写入vector的连续内存块
    octree_planes_binary_out.write(
        reinterpret_cast<char *>(octree_planes_binary_out_buffer.data()),
        static_cast<std::streamsize>(octree_planes_binary_out_buffer.size()));

    // 检查是否写入成功
    if (!octree_planes_binary_out.good()) {
      std::cerr << "unable to write file!" << std::endl;
      return;
    }
    octree_planes_binary_out.close();
  } else {
    std::cerr << "unable to open file!" << std::endl;
    return;
  }

  //打印耗时
  clock_t end_create_octree = clock();
  double create_octree_time =
      ((double)(end_create_octree - start_create_octree) /
       (double)CLOCKS_PER_SEC);
  std::cout << "create_octree_time: " << create_octree_time << std::endl;

  //平面特征可视化，发布ros话题
  Eigen::Matrix4f map_view_transformation_matrix = Eigen::Matrix4f::Identity();
  ViewPlane(octree_planes_map_select_, pub_plane_point_cloud_map_,
            pub_plane_normal_map_, map_view_transformation_matrix);

  //八叉树占用地图可视化，发布ros话题
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_occupy_map(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  plane_octree_occupy_map_depth_ = plane_octree_occupy_map_->GetOctreeDepth();
  plane_octree_occupy_map_->OctreeOccupyView(
      plane_octree_occupy_map_->point_cloud_octree_data_,
      point_cloud_occupy_map, plane_octree_occupy_map_depth_ + 1);
  sensor_msgs::PointCloud2 point_cloud_occupy_map_msg;
  point_cloud_occupy_map_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*point_cloud_occupy_map, point_cloud_occupy_map_msg);
  point_cloud_occupy_map_msg.header.frame_id = "wuba_base";
  pub_point_cloud_occupy_map_.publish(point_cloud_occupy_map_msg);
}

//读取存储在硬盘的二进制的已经构建好的八叉树地图
void ReadOctreeMap(float map_range) {
  clock_t start_read_octree = clock();

  plane_octree_occupy_map_ =
      std::make_shared<PlaneOctree>(map_range, 0.5, 20, 0.05, 0.4, 8.0, 0.35);

  //读取存储在硬盘的二进制占用八叉树地图
  std::string octree_occupy_binary_name =
      current_path_ + "/src/plane_localization/data/octree_occupy.bin";
  // 打开二进制文件
  std::ifstream octree_occupy_binary_in(octree_occupy_binary_name,
                                        std::ios::binary | std::ios::ate);
  if (!octree_occupy_binary_in.is_open()) {
    std::cerr << "unable to open file!" << std::endl;
    return;
  }
  // 获取文件大小并分配缓冲区
  std::streamsize octree_occupy_binary_in_size =
      octree_occupy_binary_in.tellg();
  octree_occupy_binary_in.seekg(0, std::ios::beg);
  // 使用 vector 作为动态缓冲区
  std::vector<char> octree_occupy_binary_in_buffer(
      octree_occupy_binary_in_size);
  // 读取整个文件到缓冲区
  if (!octree_occupy_binary_in.read(octree_occupy_binary_in_buffer.data(),
                                    octree_occupy_binary_in_size)) {
    std::cerr << "failed to read file!" << std::endl;
    return;
  }
  octree_occupy_binary_in.close();
  char *octree_occupy_binary_ptr = octree_occupy_binary_in_buffer.data();
  char *octree_occupy_binary_end_ptr =
      octree_occupy_binary_in_buffer.data() + octree_occupy_binary_in_size;
  octree_occupy_binary_ptr += sizeof(bool);
  plane_octree_occupy_map_->SetOctreeOccupyBinaryPtr(
      octree_occupy_binary_ptr, octree_occupy_binary_end_ptr);
  plane_octree_occupy_map_->BinaryToOctreeOccupy(
      plane_octree_occupy_map_->point_cloud_octree_data_);

  //读取存储在硬盘的二进制地图平特征
  std::string octree_planes_binary_name =
      current_path_ + "/src/plane_localization/data/octree_planes.bin";
  // 打开二进制文件
  std::ifstream octree_planes_binary_in(octree_planes_binary_name,
                                        std::ios::binary | std::ios::ate);
  if (!octree_planes_binary_in.is_open()) {
    std::cerr << "unable to open file!" << std::endl;
    return;
  }
  // 获取文件大小并分配缓冲区
  std::streamsize octree_planes_binary_in_size =
      octree_planes_binary_in.tellg();
  octree_planes_binary_in.seekg(0, std::ios::beg);
  // 使用 vector 作为动态缓冲区
  std::vector<char> octree_planes_binary_in_buffer(
      octree_planes_binary_in_size);
  // 读取整个文件到缓冲区
  if (!octree_planes_binary_in.read(octree_planes_binary_in_buffer.data(),
                                    octree_planes_binary_in_size)) {
    std::cerr << "failed to read file!" << std::endl;
    return;
  }
  octree_planes_binary_in.close();
  char *octree_planes_binary_ptr = octree_planes_binary_in_buffer.data();
  char *octree_planes_binary_end_ptr =
      octree_planes_binary_in_buffer.data() + octree_planes_binary_in_size;
  plane_octree_occupy_map_->SetOctreePlanesBinaryPtr(
      octree_planes_binary_ptr, octree_planes_binary_end_ptr);
  plane_octree_occupy_map_->BinaryToOctreePlanes(
      octree_planes_map_, octree_planes_binary_in_buffer);

  //按map_plane_select_rules_的设置选取平面特征
  SelectMapPlane(octree_planes_map_, octree_planes_map_select_,
                 octree_planes_map_select_index_, map_plane_select_rules_);

  //打印耗时
  clock_t end_read_octree = clock();
  double read_octree_time =
      ((double)(end_read_octree - start_read_octree) / (double)CLOCKS_PER_SEC);
  std::cout << "read_octree_time: " << read_octree_time << std::endl;

  //平面特征可视化，发布ros话题
  Eigen::Matrix4f map_view_transformation_matrix = Eigen::Matrix4f::Identity();
  ViewPlane(octree_planes_map_select_, pub_plane_point_cloud_map_,
            pub_plane_normal_map_, map_view_transformation_matrix);

  //八叉树占用地图可视化，发布ros话题
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_occupy_map(
      new pcl::PointCloud<pcl::PointXYZRGB>);
  plane_octree_occupy_map_depth_ = plane_octree_occupy_map_->GetOctreeDepth();
  plane_octree_occupy_map_->OctreeOccupyView(
      plane_octree_occupy_map_->point_cloud_octree_data_,
      point_cloud_occupy_map, plane_octree_occupy_map_depth_ + 1);
  sensor_msgs::PointCloud2 point_cloud_occupy_map_msg;
  point_cloud_occupy_map_msg.header.stamp = ros::Time::now();
  pcl::toROSMsg(*point_cloud_occupy_map, point_cloud_occupy_map_msg);
  point_cloud_occupy_map_msg.header.frame_id = "wuba_base";
  pub_point_cloud_occupy_map_.publish(point_cloud_occupy_map_msg);
}

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "plane_localization");
  ros::NodeHandle nh;

  //读参数
  ReadParam(nh);

  //定义工作空间
  const char *home_dir = getenv("HOME");
  current_path_ = home_dir;
  current_path_ = current_path_ + relative_path_;
  std::cout << "current_path_: " << current_path_ << std::endl;

  //定义ROS订阅和发布
  RegisterSubscribeAndPublish(nh);

  //读位姿数据，同时估计八叉树地图的跨度
  std::string pose_name =
      current_path_ + "/src/plane_localization/data/pose.json";
  std::cout << "pose_name: " << pose_name << std::endl;
  float map_range;
  ReadPose(pose_name, map_range);
  std::cout << "map_range: " << map_range << std::endl;

  //点云地图可视化
  ViewMapPoints();

  //八叉树地图只需要构建一次，之后直接读取即可
  //构建八叉树地图
  // ConstructOctreeMap(map_range);
  //读取已经构建好的八叉树地图
  ReadOctreeMap(map_range);

  //测试基于平面特征的重定位,测试使用，若正式运行应注释，由订阅的ros话题点云数据的回调函数中使用重定位
  TestLocalization();

  ros::spin();

  return 0;
}
