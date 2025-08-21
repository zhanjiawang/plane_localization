# plane_localization
This is a ROS package for indoor global localization (relocation) based on plane octree and plane features. The test results show that the global localization (relocation) algorithm can achieve about 3.5 seconds of octree construction and global plane feature extraction (only need to run once) in a map composed of 372 keyframes, complete the reading of plane features and octree in about 0.02 seconds, and complete global localization (relocation) in about 1.0 seconds. It is a robust and fast global localization (relocation) algorithm for indoor or other structured scenes with more plane features.

## Usage
#### 1. Requirement
```
1. ROS
2. PCL
```

#### 2. Build
```
cd ${YOUR_PROJECT_DIR}
catkin_make
```

#### 3. Run
```
source ./devel/setup.bash
roslaunch plane_localization start.launch
#result
---------------------------
transformation_matrix_vector size: 49892
cluster_transformation_matrix_vector size: 19614
cluster_transformation_time: 0.034689
---------------------------
fine_verify_octree_time: 0.084577
best_transformation_score: 1865 best_transformation_matrix: 
   -0.417447     0.908311    0.0266064   -0.0180196
   -0.908634    -0.417593 -9.00216e-05     0.061221
   0.0110289   -0.0242131     0.999646    -0.130588
           0            0            0            1
localization_time: 0.772197
```
#### 4. Example
```
The above figure achieves correct global positioning (relocation) in an indoor scene of approximately 2500 square meters
```
<img width="1183" height="1051" alt="Screenshot from 2025-08-21 19-02-45" src="https://github.com/user-attachments/assets/1d5cfc66-71d8-4679-8657-56e9c63406a5" />
