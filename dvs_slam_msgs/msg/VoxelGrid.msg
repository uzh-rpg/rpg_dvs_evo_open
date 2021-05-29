# This message contains a stamped voxel grid (3D array containing
# the number of votes for each voxel in a non-uniform, projective grid)

geometry_msgs/Pose T_w_ref             # Transformation from voxel grid coordinate system to world coordinate system
std_msgs/Float32MultiArray voxel_grid  # 3D array containing the number of votes for each cell
