# Simulate LiDAR data
Simulate LiDAR point cloud from RGBD dataset with various options such as scanning 3D model, Gaussian noise, LiDAR Fov, LiDAR interval angle etc.  

# description
LiDAR point cloud generation.</br>
<p align="center">
<img height="180" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/image.JPG"/>
<img height="180" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/result1.JPG"/>
<img height="180" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/result2.JPG"/><br/>
<img height="300" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/output1.JPG"/>
<img height="300" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/output2.JPG"/>
<p align="center">input=model_complex.obj, fov=60, interval=20, range=20.0, noise(gaussian)=0.01</p>
</p>
<p align="center">
<img height="240" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/output3.JPG"/><br/>
<img height="230" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/check1.JPG"/>
<img height="230" src="https://github.com/mac999/simulate_LiDAR/blob/main/doc/check2.JPG"/>
<p align="center">Left figure. input=model_complex1.obj, fov=180, interval=100, range=5.0, noise(gaussian)=0.05. Right figure. noise(uniform)=0.05</p></br>
</p>

# version history
v0.1</br>
> LiDAR point cloud generation draft version.</br>

v0.2</br>
> 2023.12.5., refactoring. 2023.12.16., noise option(gaussian | uniform) viewer option. test. fixed bug. update.

# install
git clone https://github.com/mac999/simulate_LiDAR.git</br>
pip install traceback, tqdm, numpy</br>
pip install trimesh</br>
pip install open3d</br>

# run
python simulate_LiDAR.py [options]</br></br>
--input: default='model.obj', help='input mesh model file(.obj, .ply)'</br>
--output: default='output.pcd', help='output file(.pcd)'</br>
--pos: default='0.0,0.0,0.0', help='LiDAR position'</br>
--yaw: default=0.0, help='LiDAR yaw angle'</br>
--fov: default=60.0, help='LiDAR field of view'</br>
--range: default=10.0, help='LiDAR range'</br>
--noise_option: default='uniform', help='Noise option. [gaussian | uniform]</br>
--noise', default=0.2, help='Noise level. ex) sigma = 0.05 in Gaussian standard deviation or uniform range'</br>
--interval: default=100, help='LiDAR interval count'</br>
--interval_angle: default=0.0, help='LiDAR interval angle'</br>
--viewer: default='on: help='run viewer = [on | off]'</br>
</br>

# license
MIT license</br>
