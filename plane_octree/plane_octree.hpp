#ifndef PLANE_OCTREE_HPP_
#define PLANE_OCTREE_HPP_

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <iostream>
#include <memory>
#include <string>
#include <vector>

typedef pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
    ColorHandlerT;

typedef struct OctreePoint {
  float x;
  float y;
  float z;
} OctreePoint;

typedef struct OctreePointCloud {
  int r;
  int g;
  int b;
  std::vector<OctreePoint> octree_point_cloud;
} OctreePointCloud;

typedef struct OctreePlane {
  bool is_allocate = false;
  int plane_area = 0;
  int plane_size = 0;
  Eigen::Vector3f plane_center = Eigen::Vector3f(0, 0, 0);
  Eigen::Vector3f plane_normal = Eigen::Vector3f(0, 0, 0);
  std::shared_ptr<OctreePointCloud> plane_data = nullptr;
  std::shared_ptr<OctreePointCloud> patch_data = nullptr;
} OctreePlane;

typedef struct OctreeNode {
  int depth = 1;
  bool occupy = false;
  Eigen::Vector3f center = Eigen::Vector3f(0, 0, 0);
  std::shared_ptr<OctreePointCloud> data = nullptr;
  std::vector<std::shared_ptr<OctreePlane>> octree_planes;
  std::vector<std::shared_ptr<OctreeNode>> octree_ptr =
      std::vector<std::shared_ptr<OctreeNode>>(8, nullptr);
} OctreeNode;

class PlaneOctree {
 public:
  PlaneOctree(float octree_range, float octree_resolution,
              int vaild_leaf_node_size,
              float plane_judgment_difference_threshold,
              float plane_judgment_similarity_threshold,
              float plane_fusion_angle_threshold,
              float plane_fusion_distance_threshold) {
    direction_vector_ = std::vector<std::vector<std::vector<int>>>(
        3, std::vector<std::vector<int>>(3, std::vector<int>(3, -1)));
    direction_vector_[0][2][2] = 0;
    direction_vector_[2][2][2] = 1;
    direction_vector_[0][0][2] = 2;
    direction_vector_[2][0][2] = 3;
    direction_vector_[0][2][0] = 4;
    direction_vector_[2][2][0] = 5;
    direction_vector_[0][0][0] = 6;
    direction_vector_[2][0][0] = 7;

    octree_range_ = octree_range;
    octree_resolution_ = octree_resolution;
    vaild_leaf_node_size_ = vaild_leaf_node_size;
    plane_judgment_difference_threshold_ = plane_judgment_difference_threshold;
    plane_judgment_similarity_threshold_ = plane_judgment_similarity_threshold;
    plane_fusion_angle_threshold_ = plane_fusion_angle_threshold;
    plane_fusion_distance_threshold_ = plane_fusion_distance_threshold;

    //数据初始化
    point_cloud_octree_data_ = std::make_shared<OctreeNode>();

    //确定八叉树所需要的深度
    octree_resolution_ = octree_resolution_ / 2.0;
    while ((octree_resolution_ * pow(2, octree_depth_)) < octree_range_) {
      octree_depth_++;
    }
    std::cout << "------------------------------------------" << std::endl;
    std::cout << "octree_range_: " << octree_range_ << std::endl;
    std::cout << "octree_resolution: " << octree_resolution_ << std::endl;
    std::cout << "vaild_leaf_node_size: " << vaild_leaf_node_size_ << std::endl;
    std::cout << "plane_judgment_difference_threshold: "
              << plane_judgment_difference_threshold_ << std::endl;
    std::cout << "plane_judgment_similarity_threshold: "
              << plane_judgment_similarity_threshold_ << std::endl;
    std::cout << "plane_fusion_angle_threshold: "
              << plane_fusion_angle_threshold_ << std::endl;
    std::cout << "plane_fusion_distance_threshold: "
              << plane_fusion_distance_threshold_ << std::endl;
    std::cout << "octree_depth: " << octree_depth_ << std::endl;
  }

  ~PlaneOctree() {}

  void ResetPlaneOctree() {
    //数据初始化
    point_cloud_octree_data_ = std::make_shared<OctreeNode>();
  }

  //计算两法向量的夹角
  float CalculateAngel(Eigen::Vector3f plane_normal_i,
                       Eigen::Vector3f plane_normal_j) {
    float plane_normal_i_dot_plane_normal_j =
        plane_normal_i.dot(plane_normal_j);
    float cos_theta = plane_normal_i_dot_plane_normal_j /
                      ((plane_normal_i.norm()) * (plane_normal_j.norm()));
    float theta = 180 * acos(cos_theta) / M_PI;
    return theta;
  }

  //计算平面中心到另一平面的距离
  float CalculateDistance(Eigen::Vector3f plane_center_i,
                          Eigen::Vector3f plane_center_j,
                          Eigen::Vector3f plane_normal_j) {
    float plane_a = plane_normal_j[0];
    float plane_b = plane_normal_j[1];
    float plane_c = plane_normal_j[2];
    float plane_d =
        -(plane_a * plane_center_j[0] + plane_b * plane_center_j[1] +
          plane_c * plane_center_j[2]);
    float distance =
        fabs(plane_a * plane_center_i[0] + plane_b * plane_center_i[1] +
             plane_c * plane_center_i[2] + plane_d) /
        sqrt(pow(plane_a, 2) + pow(plane_b, 2) + pow(plane_c, 2));
    return distance;
  }

  int GetOctreeDepth() { return octree_depth_; }

  bool QueryOccupancy(Eigen::Vector3f query_point, int depth) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    if (fabs(query_point[0]) >= octree_range_ ||
        fabs(query_point[1]) >= octree_range_ ||
        fabs(query_point[2]) >= octree_range_) {
      return false;
    }

    while (point_cloud_octree_ptr->depth < depth) {
      int direction_x =
          query_point[0] > point_cloud_octree_ptr->center[0] ? 1 : -1;
      int direction_y =
          query_point[1] > point_cloud_octree_ptr->center[1] ? 1 : -1;
      int direction_z =
          query_point[2] > point_cloud_octree_ptr->center[2] ? 1 : -1;
      int direction_index =
          direction_vector_[direction_x + 1][direction_y + 1][direction_z + 1];
      if (point_cloud_octree_ptr->octree_ptr[direction_index] != nullptr) {
        if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
            depth) {
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->occupy ==
              true) {
            return true;
          } else {
            return false;
          }
        }
        //继续到更深的节点去
        point_cloud_octree_ptr =
            point_cloud_octree_ptr->octree_ptr[direction_index];
      } else {
        return false;
      }
    }
  }

  int QueryOccupancys(pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud,
                      int depth) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    int count_occupancy = 0;
    for (auto &point : *point_cloud) {
      if (fabs(point.x) >= octree_range_ || fabs(point.y) >= octree_range_ ||
          fabs(point.z) >= octree_range_) {
        continue;
      }
      point_cloud_octree_ptr = point_cloud_octree_data_;
      while (point_cloud_octree_ptr->depth < depth) {
        int direction_x = point.x > point_cloud_octree_ptr->center[0] ? 1 : -1;
        int direction_y = point.y > point_cloud_octree_ptr->center[1] ? 1 : -1;
        int direction_z = point.z > point_cloud_octree_ptr->center[2] ? 1 : -1;
        int direction_index =
            direction_vector_[direction_x + 1][direction_y + 1]
                             [direction_z + 1];
        if (point_cloud_octree_ptr->octree_ptr[direction_index] != nullptr) {
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              depth) {
            if (point_cloud_octree_ptr->octree_ptr[direction_index]->occupy ==
                true) {
              count_occupancy++;
            }
          }
          //继续到更深的节点去
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        } else {
          break;
        }
      }
    }
    return count_occupancy;
  }

  //添加点云到八叉树
  void OctreePlaneAddPointCloud(
      pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    //把每一个点加入八叉树的同时构建八叉树
    for (auto &point : *point_cloud) {
      if (fabs(point.x) >= octree_range_ || fabs(point.y) >= octree_range_ ||
          fabs(point.z) >= octree_range_) {
        continue;
      }

      point_cloud_octree_ptr = point_cloud_octree_data_;
      while (point_cloud_octree_ptr->depth <= octree_depth_) {
        int direction_x = point.x > point_cloud_octree_ptr->center[0] ? 1 : -1;
        int direction_y = point.y > point_cloud_octree_ptr->center[1] ? 1 : -1;
        int direction_z = point.z > point_cloud_octree_ptr->center[2] ? 1 : -1;
        int direction_index =
            direction_vector_[direction_x + 1][direction_y + 1]
                             [direction_z + 1];
        //如果还没有构建此节点那么就构建此节点
        if (point_cloud_octree_ptr->octree_ptr[direction_index] == nullptr) {
          point_cloud_octree_ptr->octree_ptr[direction_index] =
              std::make_shared<OctreeNode>();
          point_cloud_octree_ptr->octree_ptr[direction_index]->occupy = true;

          point_cloud_octree_ptr->octree_ptr[direction_index]->depth =
              point_cloud_octree_ptr->depth + 1;
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[0] =
              point_cloud_octree_ptr->center[0] +
              (direction_x * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[1] =
              point_cloud_octree_ptr->center[1] +
              (direction_y * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[2] =
              point_cloud_octree_ptr->center[2] +
              (direction_z * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));

          //如果到了最深的深度也就是叶子节点那么就把点数据放入叶子节点
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              (octree_depth_ + 1)) {
            //在这个深度上它的八个子节点是nullptr但是data是有数据的
            if (point_cloud_octree_ptr->octree_ptr[direction_index]->data ==
                nullptr) {
              point_cloud_octree_ptr->octree_ptr[direction_index]->data =
                  std::make_shared<OctreePointCloud>();
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->r =
                  rand() % 255;
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->g =
                  rand() % 255;
              point_cloud_octree_ptr->octree_ptr[direction_index]->data->b =
                  rand() % 255;
              OctreePoint odtree_point;
              odtree_point.x = point.x;
              odtree_point.y = point.y;
              odtree_point.z = point.z;
              point_cloud_octree_ptr->octree_ptr[direction_index]
                  ->data->octree_point_cloud.push_back(odtree_point);
            } else {
              //已经构造过叶子节点了，直接放入数据
              OctreePoint odtree_point;
              odtree_point.x = point.x;
              odtree_point.y = point.y;
              odtree_point.z = point.z;
              point_cloud_octree_ptr->octree_ptr[direction_index]
                  ->data->octree_point_cloud.push_back(odtree_point);
            }
          }

          //继续到更深的节点去
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        } else {
          //已经构建过此节点了，判断它是不是叶子节点，是的话直接放入数据
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              (octree_depth_ + 1)) {
            OctreePoint odtree_point;
            odtree_point.x = point.x;
            odtree_point.y = point.y;
            odtree_point.z = point.z;
            point_cloud_octree_ptr->octree_ptr[direction_index]
                ->data->octree_point_cloud.push_back(odtree_point);
          }

          //继续到更深的节点去
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        }
      }
    }
  }

  //添加平面到八叉树
  void OctreePlaneAddPlanes(
      std::vector<std::shared_ptr<OctreePlane>> &octree_planes) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    //把每一个平面加入八叉树的同时构建八叉树
    for (int i = 0; i < octree_planes.size(); i++) {
      if (fabs(octree_planes[i]->plane_center[0]) >= octree_range_ ||
          fabs(octree_planes[i]->plane_center[1]) >= octree_range_ ||
          fabs(octree_planes[i]->plane_center[2]) >= octree_range_) {
        continue;
      }

      point_cloud_octree_ptr = point_cloud_octree_data_;
      while (point_cloud_octree_ptr->depth <= octree_depth_) {
        int direction_x = (octree_planes[i]->plane_center[0]) >
                                  point_cloud_octree_ptr->center[0]
                              ? 1
                              : -1;
        int direction_y = (octree_planes[i]->plane_center[1]) >
                                  point_cloud_octree_ptr->center[1]
                              ? 1
                              : -1;
        int direction_z = (octree_planes[i]->plane_center[2]) >
                                  point_cloud_octree_ptr->center[2]
                              ? 1
                              : -1;
        int direction_index =
            direction_vector_[direction_x + 1][direction_y + 1]
                             [direction_z + 1];
        //如果还没有构建此节点那么就构建此节点
        if (point_cloud_octree_ptr->octree_ptr[direction_index] == nullptr) {
          point_cloud_octree_ptr->octree_ptr[direction_index] =
              std::make_shared<OctreeNode>();
          point_cloud_octree_ptr->octree_ptr[direction_index]->depth =
              point_cloud_octree_ptr->depth + 1;
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[0] =
              point_cloud_octree_ptr->center[0] +
              (direction_x * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[1] =
              point_cloud_octree_ptr->center[1] +
              (direction_y * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
          point_cloud_octree_ptr->octree_ptr[direction_index]->center[2] =
              point_cloud_octree_ptr->center[2] +
              (direction_z * octree_resolution_ *
               pow(2, octree_depth_ - point_cloud_octree_ptr->depth));

          //如果到了最深的深度也就是叶子节点那么就把点数据放入叶子节点
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              (octree_depth_ + 1)) {
            point_cloud_octree_ptr->octree_ptr[direction_index]
                ->octree_planes.push_back(octree_planes[i]);
          }

          //继续到更深的节点去
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        } else {
          //已经构建过此节点了，判断它是不是叶子节点，是的话直接放入数据
          if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
              (octree_depth_ + 1)) {
            point_cloud_octree_ptr->octree_ptr[direction_index]
                ->octree_planes.push_back(octree_planes[i]);
          }

          //继续到更深的节点去
          point_cloud_octree_ptr =
              point_cloud_octree_ptr->octree_ptr[direction_index];
        }
      }
    }
  }

  //重新计算平面的面积
  void GetOctreePlaneArea(
      std::vector<std::shared_ptr<OctreePlane>> &octree_planes) {
    for (int i = 0; i < octree_planes.size(); i++) {
      point_cloud_octree_data_ = std::make_shared<OctreeNode>();
      std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
          point_cloud_octree_data_;
      int plane_area = 0;
      for (int j = 0;
           j < octree_planes[i]->patch_data->octree_point_cloud.size(); j++) {
        if (fabs(octree_planes[i]->patch_data->octree_point_cloud[j].x) >=
                octree_range_ ||
            fabs(octree_planes[i]->patch_data->octree_point_cloud[j].y) >=
                octree_range_ ||
            fabs(octree_planes[i]->patch_data->octree_point_cloud[j].z) >=
                octree_range_) {
          continue;
        }

        point_cloud_octree_ptr = point_cloud_octree_data_;
        while (point_cloud_octree_ptr->depth <= octree_depth_) {
          int direction_x =
              (octree_planes[i]->patch_data->octree_point_cloud[j].x) >
                      point_cloud_octree_ptr->center[0]
                  ? 1
                  : -1;
          int direction_y =
              (octree_planes[i]->patch_data->octree_point_cloud[j].y) >
                      point_cloud_octree_ptr->center[1]
                  ? 1
                  : -1;
          int direction_z =
              (octree_planes[i]->patch_data->octree_point_cloud[j].z) >
                      point_cloud_octree_ptr->center[2]
                  ? 1
                  : -1;
          int direction_index =
              direction_vector_[direction_x + 1][direction_y + 1]
                               [direction_z + 1];
          //如果还没有构建此节点那么就构建此节点
          if (point_cloud_octree_ptr->octree_ptr[direction_index] == nullptr) {
            point_cloud_octree_ptr->octree_ptr[direction_index] =
                std::make_shared<OctreeNode>();
            point_cloud_octree_ptr->octree_ptr[direction_index]->depth =
                point_cloud_octree_ptr->depth + 1;
            point_cloud_octree_ptr->octree_ptr[direction_index]->center[0] =
                point_cloud_octree_ptr->center[0] +
                (direction_x * octree_resolution_ *
                 pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
            point_cloud_octree_ptr->octree_ptr[direction_index]->center[1] =
                point_cloud_octree_ptr->center[1] +
                (direction_y * octree_resolution_ *
                 pow(2, octree_depth_ - point_cloud_octree_ptr->depth));
            point_cloud_octree_ptr->octree_ptr[direction_index]->center[2] =
                point_cloud_octree_ptr->center[2] +
                (direction_z * octree_resolution_ *
                 pow(2, octree_depth_ - point_cloud_octree_ptr->depth));

            //如果到了最深的深度也就是叶子节点那么就把点数据放入叶子节点
            if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
                (octree_depth_ + 1)) {
              plane_area++;
              point_cloud_octree_ptr->octree_ptr[direction_index]->occupy =
                  true;
            }

            //继续到更深的节点去
            point_cloud_octree_ptr =
                point_cloud_octree_ptr->octree_ptr[direction_index];
          } else {
            //已经构建过此节点了，判断它是不是叶子节点，是的话直接放入数据
            if (point_cloud_octree_ptr->octree_ptr[direction_index]->depth ==
                (octree_depth_ + 1)) {
              if (point_cloud_octree_ptr->octree_ptr[direction_index]->occupy ==
                  false) {
                plane_area++;
                point_cloud_octree_ptr->octree_ptr[direction_index]->occupy =
                    true;
              }
            }

            //继续到更深的节点去
            point_cloud_octree_ptr =
                point_cloud_octree_ptr->octree_ptr[direction_index];
          }
        }
      }
      octree_planes[i]->plane_area = plane_area;
    }
  }

  //将平面特征转为二进制
  void OctreePlanesToBinary(
      std::vector<std::shared_ptr<OctreePlane>> &octree_planes,
      std::vector<char> &octree_occupy_binary_buffer) {
    for (int i = 0; i < octree_planes.size(); i++) {
      int plane_area = octree_planes[i]->plane_area;
      char *plane_area_ptr = reinterpret_cast<char *>(&plane_area);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         plane_area_ptr,
                                         plane_area_ptr + sizeof(int));
      float plane_center_x = octree_planes[i]->plane_center[0];
      char *plane_center_x_ptr = reinterpret_cast<char *>(&plane_center_x);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         plane_center_x_ptr,
                                         plane_center_x_ptr + sizeof(float));
      float plane_center_y = octree_planes[i]->plane_center[1];
      char *plane_center_y_ptr = reinterpret_cast<char *>(&plane_center_y);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         plane_center_y_ptr,
                                         plane_center_y_ptr + sizeof(float));
      float plane_center_z = octree_planes[i]->plane_center[2];
      char *plane_center_z_ptr = reinterpret_cast<char *>(&plane_center_z);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         plane_center_z_ptr,
                                         plane_center_z_ptr + sizeof(float));
      float plane_normal_x = octree_planes[i]->plane_normal[0];
      char *plane_normal_x_ptr = reinterpret_cast<char *>(&plane_normal_x);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         plane_normal_x_ptr,
                                         plane_normal_x_ptr + sizeof(float));
      float plane_normal_y = octree_planes[i]->plane_normal[1];
      char *plane_normal_y_ptr = reinterpret_cast<char *>(&plane_normal_y);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         plane_normal_y_ptr,
                                         plane_normal_y_ptr + sizeof(float));
      float plane_normal_z = octree_planes[i]->plane_normal[2];
      char *plane_normal_z_ptr = reinterpret_cast<char *>(&plane_normal_z);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         plane_normal_z_ptr,
                                         plane_normal_z_ptr + sizeof(float));
      int red = octree_planes[i]->plane_data->r;
      char *red_ptr = reinterpret_cast<char *>(&red);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         red_ptr, red_ptr + sizeof(int));
      int green = octree_planes[i]->plane_data->g;
      char *green_ptr = reinterpret_cast<char *>(&green);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         green_ptr, green_ptr + sizeof(int));
      int blue = octree_planes[i]->plane_data->b;
      char *blue_ptr = reinterpret_cast<char *>(&blue);
      octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                         blue_ptr, blue_ptr + sizeof(int));
    }
  }

  void SetOctreePlanesBinaryPtr(char *octree_planes_binary_ptr,
                                char *octree_planes_binary_end_ptr) {
    octree_planes_binary_ptr_ = octree_planes_binary_ptr;
    octree_planes_binary_end_ptr_ = octree_planes_binary_end_ptr;
  }

  //将二进制转为平面特征
  void BinaryToOctreePlanes(
      std::vector<std::shared_ptr<OctreePlane>> &octree_planes,
      std::vector<char> &octree_planes_binary_buffer) {
    while (octree_planes_binary_ptr_ < octree_planes_binary_end_ptr_) {
      int plane_area = *reinterpret_cast<int *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(int);
      float plane_center_x =
          *reinterpret_cast<float *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(float);
      float plane_center_y =
          *reinterpret_cast<float *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(float);
      float plane_center_z =
          *reinterpret_cast<float *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(float);
      float plane_normal_x =
          *reinterpret_cast<float *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(float);
      float plane_normal_y =
          *reinterpret_cast<float *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(float);
      float plane_normal_z =
          *reinterpret_cast<float *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(float);
      int red = *reinterpret_cast<int *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(int);
      int green = *reinterpret_cast<int *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(int);
      int blue = *reinterpret_cast<int *>(octree_planes_binary_ptr_);
      octree_planes_binary_ptr_ += sizeof(int);

      std::shared_ptr<OctreePlane> octree_plane =
          std::make_shared<OctreePlane>();
      octree_plane->plane_area = plane_area;
      octree_plane->plane_center[0] = plane_center_x;
      octree_plane->plane_center[1] = plane_center_y;
      octree_plane->plane_center[2] = plane_center_z;
      octree_plane->plane_normal[0] = plane_normal_x;
      octree_plane->plane_normal[1] = plane_normal_y;
      octree_plane->plane_normal[2] = plane_normal_z;
      octree_plane->plane_data = std::make_shared<OctreePointCloud>();
      octree_plane->plane_data->r = red;
      octree_plane->plane_data->g = green;
      octree_plane->plane_data->b = blue;
      OctreePoint odtree_point;
      odtree_point.x = plane_center_x;
      odtree_point.y = plane_center_y;
      odtree_point.z = plane_center_z;
      octree_plane->plane_data->octree_point_cloud.push_back(odtree_point);

      octree_planes.push_back(octree_plane);
    }
  }

  //可视化八叉树的叶子节点占用情况
  void OctreeOccupyView(
      std::shared_ptr<OctreeNode> point_cloud_octree_ptr,
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr point_cloud_occupy, int depth) {
    if (point_cloud_octree_ptr->depth == depth &&
        point_cloud_octree_ptr->occupy == true) {
      pcl::PointXYZRGB point_xyzrgb;
      point_xyzrgb.x = point_cloud_octree_ptr->center[0];
      point_xyzrgb.y = point_cloud_octree_ptr->center[1];
      point_xyzrgb.z = point_cloud_octree_ptr->center[2];
      point_xyzrgb.r = rand() % 255;
      point_xyzrgb.g = rand() % 255;
      point_xyzrgb.b = rand() % 255;
      (*point_cloud_occupy).push_back(point_xyzrgb);
    }
    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreeOccupyView(point_cloud_octree_ptr->octree_ptr[i],
                         point_cloud_occupy, depth);
      }
    }
  }

  //将占用八叉树存储为二进制文件
  void OctreeOccupyToBinary(std::shared_ptr<OctreeNode> point_cloud_octree_ptr,
                            std::vector<char> &octree_occupy_binary_buffer) {
    bool ptr_not_nullptr = true;
    char *ptr_not_nullptr_ptr = reinterpret_cast<char *>(&ptr_not_nullptr);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       ptr_not_nullptr_ptr,
                                       ptr_not_nullptr_ptr + sizeof(bool));
    int depth = point_cloud_octree_ptr->depth;
    char *depth_ptr = reinterpret_cast<char *>(&depth);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       depth_ptr, depth_ptr + sizeof(int));
    bool occupy = point_cloud_octree_ptr->occupy;
    char *occupy_ptr = reinterpret_cast<char *>(&occupy);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       occupy_ptr, occupy_ptr + sizeof(bool));
    float center_x = point_cloud_octree_ptr->center[0];
    char *center_x_ptr = reinterpret_cast<char *>(&center_x);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       center_x_ptr,
                                       center_x_ptr + sizeof(float));
    float center_y = point_cloud_octree_ptr->center[1];
    char *center_y_ptr = reinterpret_cast<char *>(&center_y);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       center_y_ptr,
                                       center_y_ptr + sizeof(float));
    float center_z = point_cloud_octree_ptr->center[2];
    char *center_z_ptr = reinterpret_cast<char *>(&center_z);
    octree_occupy_binary_buffer.insert(octree_occupy_binary_buffer.end(),
                                       center_z_ptr,
                                       center_z_ptr + sizeof(float));

    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreeOccupyToBinary(point_cloud_octree_ptr->octree_ptr[i],
                             octree_occupy_binary_buffer);
      } else {
        bool child_ptr_not_nullptr = false;
        char *child_ptr_not_nullptr_ptr =
            reinterpret_cast<char *>(&child_ptr_not_nullptr);
        octree_occupy_binary_buffer.insert(
            octree_occupy_binary_buffer.end(), child_ptr_not_nullptr_ptr,
            child_ptr_not_nullptr_ptr + sizeof(bool));
      }
    }
  }

  void SetOctreeOccupyBinaryPtr(char *octree_occupy_binary_ptr,
                                char *octree_occupy_binary_end_ptr) {
    octree_occupy_binary_ptr_ = octree_occupy_binary_ptr;
    octree_occupy_binary_end_ptr_ = octree_occupy_binary_end_ptr;
  }

  //将二进制文件解析为占用八叉树
  void BinaryToOctreeOccupy(
      std::shared_ptr<OctreeNode> point_cloud_octree_ptr) {
    if (octree_occupy_binary_ptr_ < octree_occupy_binary_end_ptr_) {
      int depth = *reinterpret_cast<int *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->depth = depth;
      octree_occupy_binary_ptr_ += sizeof(int);
      bool occupy = *reinterpret_cast<bool *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->occupy = occupy;
      octree_occupy_binary_ptr_ += sizeof(bool);
      float center_x = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->center[0] = center_x;
      octree_occupy_binary_ptr_ += sizeof(float);
      float center_y = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->center[1] = center_y;
      octree_occupy_binary_ptr_ += sizeof(float);
      float center_z = *reinterpret_cast<float *>(octree_occupy_binary_ptr_);
      point_cloud_octree_ptr->center[2] = center_z;
      octree_occupy_binary_ptr_ += sizeof(float);
      for (int i = 0; i < 8; ++i) {
        if (octree_occupy_binary_ptr_ < octree_occupy_binary_end_ptr_) {
          bool not_nullptr =
              *reinterpret_cast<bool *>(octree_occupy_binary_ptr_);
          octree_occupy_binary_ptr_ += sizeof(bool);
          if (not_nullptr) {
            point_cloud_octree_ptr->octree_ptr[i] =
                std::make_shared<OctreeNode>();
            BinaryToOctreeOccupy(point_cloud_octree_ptr->octree_ptr[i]);
          }
        }
      }
    }
  }

  //判断八叉树的叶子节点是否是平面
  void OctreePlaneJudgment(std::shared_ptr<OctreeNode> point_cloud_octree_ptr) {
    if (point_cloud_octree_ptr->data != nullptr) {
      int left_node_point_cloud_size =
          point_cloud_octree_ptr->data->octree_point_cloud.size();
      if (left_node_point_cloud_size > vaild_leaf_node_size_) {
        Eigen::Vector3f centroid(0, 0, 0);
        for (int i = 0; i < left_node_point_cloud_size; i++) {
          Eigen::Vector3f point(
              point_cloud_octree_ptr->data->octree_point_cloud[i].x,
              point_cloud_octree_ptr->data->octree_point_cloud[i].y,
              point_cloud_octree_ptr->data->octree_point_cloud[i].z);
          centroid += point;
        }
        centroid /= left_node_point_cloud_size;

        Eigen::Matrix3f covariance = Eigen::Matrix3f::Zero();
        for (int i = 0; i < left_node_point_cloud_size; i++) {
          Eigen::Vector3f point(
              point_cloud_octree_ptr->data->octree_point_cloud[i].x,
              point_cloud_octree_ptr->data->octree_point_cloud[i].y,
              point_cloud_octree_ptr->data->octree_point_cloud[i].z);
          Eigen::Vector3f centered = point - centroid;
          covariance += centered * centered.transpose();
        }
        covariance /= left_node_point_cloud_size;

        Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> solver(covariance);

        Eigen::Vector3f eigenvalues = solver.eigenvalues();
        Eigen::Matrix3f eigenvectors = solver.eigenvectors();

        Eigen::Vector3f normal = eigenvectors.col(0);
        Eigen::Vector3f normal_opposite = -normal;

        if ((eigenvalues[0] / eigenvalues[2]) <
                plane_judgment_difference_threshold_ &&
            (eigenvalues[1] / eigenvalues[2]) >
                plane_judgment_similarity_threshold_) {
          int r = point_cloud_octree_ptr->data->r;
          int g = point_cloud_octree_ptr->data->g;
          int b = point_cloud_octree_ptr->data->b;
          std::shared_ptr<OctreePlane> octree_plane =
              std::make_shared<OctreePlane>();
          octree_plane->is_allocate = false;
          octree_plane->plane_area = 1;
          octree_plane->plane_size = left_node_point_cloud_size;
          octree_plane->plane_center = centroid;
          if (normal.dot(centroid) < 0) {
            octree_plane->plane_normal = normal;
          } else {
            octree_plane->plane_normal = normal_opposite;
          }
          if (octree_plane->plane_data == nullptr) {
            octree_plane->plane_data = std::make_shared<OctreePointCloud>();
            octree_plane->plane_data->r = r;
            octree_plane->plane_data->g = g;
            octree_plane->plane_data->b = b;
          }
          if (octree_plane->patch_data == nullptr) {
            octree_plane->patch_data = std::make_shared<OctreePointCloud>();
            OctreePoint odtree_point;
            odtree_point.x = centroid[0];
            odtree_point.y = centroid[1];
            odtree_point.z = centroid[2];
            octree_plane->patch_data->octree_point_cloud.push_back(
                odtree_point);
          }
          for (int i = 0; i < left_node_point_cloud_size; i++) {
            OctreePoint odtree_point;
            odtree_point.x =
                point_cloud_octree_ptr->data->octree_point_cloud[i].x;
            odtree_point.y =
                point_cloud_octree_ptr->data->octree_point_cloud[i].y;
            odtree_point.z =
                point_cloud_octree_ptr->data->octree_point_cloud[i].z;
            octree_plane->plane_data->octree_point_cloud.push_back(
                odtree_point);
          }
          point_cloud_octree_ptr->octree_planes.push_back(octree_plane);
        }
      }
    }
    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreePlaneJudgment(point_cloud_octree_ptr->octree_ptr[i]);
      }
    }
  }

  //融合八叉树不同深度上的平面特征
  void OctreePlaneFusion(std::shared_ptr<OctreeNode> point_cloud_octree_ptr,
                         int fusion_depth) {
    if (point_cloud_octree_ptr->depth == fusion_depth) {
      std::vector<std::shared_ptr<OctreePlane>> octree_node_planes;
      for (int i = 0; i < 8; i++) {
        if (point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
          if (point_cloud_octree_ptr->octree_ptr[i]->octree_planes.size() > 0) {
            for (int j = 0;
                 j <
                 point_cloud_octree_ptr->octree_ptr[i]->octree_planes.size();
                 j++) {
              octree_node_planes.push_back(
                  point_cloud_octree_ptr->octree_ptr[i]->octree_planes[j]);
            }
          }
        }
      }
      for (int i = 0; i < octree_node_planes.size(); i++) {
        if (octree_node_planes[i]->is_allocate == false) {
          bool have_fusion = true;
          while (have_fusion) {
            have_fusion = false;
            for (int j = 0; j < octree_node_planes.size(); j++) {
              if (i != j && octree_node_planes[j]->is_allocate == false) {
                float angle_ij =
                    CalculateAngel(octree_node_planes[i]->plane_normal,
                                   octree_node_planes[j]->plane_normal);
                float distance_i =
                    CalculateDistance(octree_node_planes[i]->plane_center,
                                      octree_node_planes[j]->plane_center,
                                      octree_node_planes[j]->plane_normal);
                float distance_j =
                    CalculateDistance(octree_node_planes[j]->plane_center,
                                      octree_node_planes[i]->plane_center,
                                      octree_node_planes[i]->plane_normal);

                if (angle_ij < plane_fusion_angle_threshold_ &&
                    distance_i < plane_fusion_distance_threshold_ &&
                    distance_j < plane_fusion_distance_threshold_) {
                  have_fusion = true;
                  octree_node_planes[j]->is_allocate = true;
                  for (int k = 0;
                       k < octree_node_planes[j]
                               ->plane_data->octree_point_cloud.size();
                       k++) {
                    OctreePoint odtree_point;
                    odtree_point.x = octree_node_planes[j]
                                         ->plane_data->octree_point_cloud[k]
                                         .x;
                    odtree_point.y = octree_node_planes[j]
                                         ->plane_data->octree_point_cloud[k]
                                         .y;
                    odtree_point.z = octree_node_planes[j]
                                         ->plane_data->octree_point_cloud[k]
                                         .z;
                    octree_node_planes[i]
                        ->plane_data->octree_point_cloud.push_back(
                            odtree_point);
                  }
                  for (int k = 0;
                       k < octree_node_planes[j]
                               ->patch_data->octree_point_cloud.size();
                       k++) {
                    OctreePoint odtree_point;
                    odtree_point.x = octree_node_planes[j]
                                         ->patch_data->octree_point_cloud[k]
                                         .x;
                    odtree_point.y = octree_node_planes[j]
                                         ->patch_data->octree_point_cloud[k]
                                         .y;
                    odtree_point.z = octree_node_planes[j]
                                         ->patch_data->octree_point_cloud[k]
                                         .z;
                    octree_node_planes[i]
                        ->patch_data->octree_point_cloud.push_back(
                            odtree_point);
                  }
                  octree_node_planes[i]->plane_center =
                      ((octree_node_planes[i]->plane_size *
                        octree_node_planes[i]->plane_center) +
                       (octree_node_planes[j]->plane_size *
                        octree_node_planes[j]->plane_center)) /
                      (octree_node_planes[i]->plane_size +
                       octree_node_planes[j]->plane_size);
                  octree_node_planes[i]->plane_normal =
                      (octree_node_planes[i]->plane_size *
                       octree_node_planes[i]->plane_normal) +
                      (octree_node_planes[j]->plane_size *
                       octree_node_planes[j]->plane_normal);
                  octree_node_planes[i]->plane_normal.normalize();
                  octree_node_planes[i]->plane_size =
                      octree_node_planes[i]->plane_size +
                      octree_node_planes[j]->plane_size;
                  octree_node_planes[i]->plane_area =
                      octree_node_planes[i]->plane_area +
                      octree_node_planes[j]->plane_area;
                }
              }
            }
          }
        }
      }
      for (int i = 0; i < octree_node_planes.size(); i++) {
        if (octree_node_planes[i]->is_allocate == false) {
          point_cloud_octree_ptr->octree_planes.push_back(
              octree_node_planes[i]);
        }
      }
    }
    for (int i = 0; i < 8; i++) {
      if (point_cloud_octree_ptr->depth < fusion_depth &&
          point_cloud_octree_ptr->octree_ptr[i] != nullptr) {
        OctreePlaneFusion(point_cloud_octree_ptr->octree_ptr[i], fusion_depth);
      }
    }
  }

  //提取八叉树的平面特征
  void OctreePlaneExtraction(
      std::vector<std::shared_ptr<OctreePlane>> &octree_planes_get,
      bool judge) {
    std::shared_ptr<OctreeNode> point_cloud_octree_ptr =
        point_cloud_octree_data_;

    if (judge == true) {
      OctreePlaneJudgment(point_cloud_octree_ptr);
    }

    //在八叉树不同深度上进行平面特征的融合
    for (int depth = octree_depth_; depth > 0; depth--) {
      point_cloud_octree_ptr = point_cloud_octree_data_;
      OctreePlaneFusion(point_cloud_octree_ptr, depth);
    }
    for (int i = 0; i < point_cloud_octree_data_->octree_planes.size(); i++) {
      octree_planes_get.push_back(point_cloud_octree_data_->octree_planes[i]);
    }
  }

 public:
  std::shared_ptr<OctreeNode> point_cloud_octree_data_;

 private:
  //八叉树八个节点的索引
  std::vector<std::vector<std::vector<int>>> direction_vector_;

  char *octree_occupy_binary_ptr_;
  char *octree_occupy_binary_end_ptr_;
  char *octree_planes_binary_ptr_;
  char *octree_planes_binary_end_ptr_;

  float octree_range_ = 50.0;
  float octree_resolution_ = 0.5;  //八叉树的分辨率
  int octree_depth_ = 1;  //八叉树的初始深度，后续根据输入点云的跨度重新计算
  int vaild_leaf_node_size_ = 20;  //有效的叶子节点的点数量
  float plane_judgment_difference_threshold_ =
      0.01;  //判断叶子节点为平面的最小特征值阈值与最大特征值的比值
  float plane_judgment_similarity_threshold_ =
      0.5;  //判断叶子节点为平面的中间征值阈值与最大特征值的比值
  float plane_fusion_angle_threshold_ =
      5.0;  //进行平面融合时的两平面的法向量夹角阈值
  float plane_fusion_distance_threshold_ =
      0.25;  //进行平面融合时的平面中心到另一平面的距离阈值
};

#endif