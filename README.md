# simulate_LiDAR
Simulate LiDAR point cloud from RGBD dataset

# description
LiDAR point cloud generation.</br>
<p align="center">
<img height="400" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/result1.JPG"/>
</p>

# run
python simulate_LiDAR.py [options]</br>
--input: default='model.obj', help='input mesh model file(.obj, .ply)'</br>
--output: default='output.pcd', help='output file(.pcd)'</br>
--pos: default='0.0,0.0,0.0', help='LiDAR position'</br>
--yaw: default='0.0', help='LiDAR yaw angle'</br>
--fov: default='60.0', help='LiDAR field of view'</br>
--range: default='10.0', help='LiDAR range'</br>

# version history
v0.1</br>
> LiDAR point cloud generation draft version.
