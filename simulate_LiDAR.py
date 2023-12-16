# title: simulate LiDAR from RGBD dataset
# author: taewook kang
# email: laputa99999@gmail.com
# version: draft. testing. 
#  0.1. updated.
# date: 
#  2023.10.1. init
#  2023.11.4. support mesh object, interval option
#  2023.12.5. refactoring 
#  2023.12.16. viewer option. test. fixed bug. update.
# 
import os, sys, argparse, json, re, math, traceback
import trimesh, traceback
import numpy as np
from tqdm import tqdm
import open3d as o3d

def create_box(extents = [5.0, 5.0, 3.0]):
	mesh = trimesh.primitives.Box(extents)
	return mesh

def load_mesh(filename):
	mesh = trimesh.load(filename)
	return mesh

def create_plane_targets(plane_y, targets_count=50):
	point_targets = []
	for i in range(targets_count):
		for j in range(targets_count):
			step = 0.1
			x = i * step - 2.5
			y = plane_y
			z = j * step - 2.5
			target = trimesh.primitives.Box(extents = [0.1, 0.1, 0.1], transform = trimesh.transformations.translation_matrix([x, y, z]))
			# target = trimesh.primitives.Plane() TBD
			point_targets.append(target)
	return point_targets

class lidar_device:
	# Define LiDAR parameters
	position = [0.0, 0.0, 0.0]  # LiDAR's position in the room
	yaw = 0.0  # LiDAR's yaw angle (rotation around the vertical axis)
	fov = 60.0 # 10 # LiDAR's field of view in degrees
	interval = 20 # Angular interval between rays in degrees
	range = 10.0  # Maximum LiDAR range
	noise = 0.2 # Noise level (Gaussian standard deviation) in meters

	def init(self, pos=[0.0,0.0,0.0], yaw=0.0, fov_angle=60.0, interval=20, range = 10, noise=0.2):
		self.position = pos
		self.yaw = yaw
		self.fov = fov_angle
		self.interval = interval
		self.range = range
		self.noise = noise

	def create_rays(self):
		ray_origins = None # np.array([[0.0, 0.5, 0.0]])
		ray_directions = None # np.array([[0.0, 0.5, 2.0]])
		
		# Generate LiDAR point cloud
		lidar_pcd = o3d.geometry.PointCloud()
		for azimuth in np.linspace(self.yaw - self.fov / 2, self.yaw + self.fov / 2, self.interval): #360):
			for elevation in np.linspace(-self.fov / 2, self.fov / 2, self.interval): # 180):
				# Compute laser beam direction in LiDAR's local coordinates
				direction = np.array([
					np.cos(np.radians(elevation)) * np.sin(np.radians(azimuth)),
					np.cos(np.radians(elevation)) * np.cos(np.radians(azimuth)),
					np.sin(np.radians(elevation)),
				])

				# Compute the end point of the laser beam
				end_point = np.array([
					self.position[0] + direction[0] * self.range,
					self.position[1] + direction[1] * self.range,
					self.position[2] + direction[2] * self.range,
				])

				if ray_origins is None:
					ray_origins = np.array([self.position])
					ray_directions = np.array([end_point - self.position])
				else:
					ray_origins = np.append(ray_origins, self.position)
					ray_directions = np.append(ray_directions, end_point - self.position)
		
		ray_origins = ray_origins.reshape(-1, 3)
		ray_directions = ray_directions.reshape(-1, 3)

		return ray_origins, ray_directions

	def scan(self, ray_origins, ray_directions, targets):
		ray_lines_set = []
		intersection_points_set = []
		for target in tqdm(targets):
			vertices = target.vertices.copy()  # Make a copy of the vertices array
			faces = target.faces.copy()        # Make a copy of the faces array
			m = o3d.geometry.TriangleMesh()
			m.vertices = o3d.utility.Vector3dVector(vertices)
			m.triangles = o3d.utility.Vector3iVector(faces)        
		
			locations, index_ray, index_tri = target.ray.intersects_location(
				ray_origins=ray_origins,
				ray_directions=ray_directions)

			if len(locations) <= 0:
				continue

			# Create an Open3D PointCloud from the intersection points
			intersection_points = o3d.geometry.PointCloud()
			locations = locations + np.random.normal(0, self.noise, locations.shape)
			intersection_points.points = o3d.utility.Vector3dVector(locations) 
			intersection_points_set.append(intersection_points)

			# Create an Open3D LineSet to visualize rays
			ray_lines = o3d.geometry.LineSet()
			ray_lines.points = o3d.utility.Vector3dVector(np.vstack((ray_origins, ray_origins + ray_directions)))
			num_points = len(ray_origins)
			line_indices = np.array([[i, i + num_points] for i in range(num_points)])
			ray_lines.lines = o3d.utility.Vector2iVector(line_indices)    # Create lines between pairs of points. index. 0,1 is the first line, 2,3 is the second line
			ray_lines_set.append(ray_lines)

		return ray_lines_set, intersection_points_set

def create_material(shader, base_color, thickness = 0.0, transmission = 0.0, absorption_color = [0.0,0.0,0.0], point_size = 0.0):
	mat = o3d.visualization.rendering.MaterialRecord()
	mat.shader = shader
	mat.base_color = base_color
	mat.base_roughness = 0.0
	mat.base_reflectance = 0.0
	mat.base_clearcoat = 1.0
	mat.thickness = thickness
	if point_size > 0.0:
		mat.point_size = point_size
	mat.transmission = transmission
	mat.absorption_distance = 10
	mat.absorption_color = absorption_color
	return mat

def convert_mesh(mesh):
	vertices = mesh.vertices.copy()  
	normals = mesh.vertex_normals
	faces = mesh.faces.copy()       
	o3d_mesh = o3d.geometry.TriangleMesh()
	o3d_mesh.vertices = o3d.utility.Vector3dVector(vertices)
	o3d_mesh.triangles = o3d.utility.Vector3iVector(faces)
	writable_normals = np.array(normals, copy=True)
	o3d_mesh.vertex_normals = o3d.utility.Vector3dVector(writable_normals)

	return o3d_mesh

def view_simulation_results(ray_lines_set, intersection_points_set, point_targets, mesh):
	geoms = []
	o3d_mesh = convert_mesh(mesh)
	mat_lit_trans = create_material('defaultLitTransparency', base_color=[0.467,0.467,0.467,0.2], thickness=1.0, transmission=0.5, absorption_color=[0.5, 0.5, 0.5])
	# mat_lit_trans = create_material('unlitSolidColor', base_color=[0.467,0.467,0.467,0.8], thickness=1.0, transmission=0.1) # , absorption_color=[0.5, 0.5, 0.5])
	geom = {'name': 'box', 'geometry': o3d_mesh, 'material': mat_lit_trans}
	geoms.append(geom)

	# materials. defaultLit, defaultUnlit, unlitLine, unlitGradient, unlitSolidColor, defaultLitTransparency.
	# http://www.open3d.org/docs/latest/python_api/open3d.visualization.tensorboard_plugin.summary.add_3d.html
	mat_lay = create_material('unlitSolidColor', base_color=[0.0, 0, 1.0, 1.0], thickness=1.0)
	for index, ray_lines in enumerate(ray_lines_set):
		name = 'line' + str(index)		
		ray = {'name': name, 'geometry': ray_lines, 'material': mat_lay}
		geoms.append(ray)
	
	mat_point = create_material('unlitSolidColor', base_color=[1.0, 0, 0, 1.0], thickness=1.0) # , point_size=10.0)	
	for index, intersection_points in enumerate(intersection_points_set):
		name = 'intpoints' + str(index)
		pt = {'name': name, 'geometry': intersection_points, 'material': mat_point}
		geoms.append(pt)

	mat_target = create_material('defaultLitTransparency', base_color=[0.0,0.5,1.0,0.5], thickness=1.0, transmission=0.5, absorption_color=[0.5, 0.5, 0.5])
	for index, m in enumerate(point_targets):
		name = 'box' + str(index)
		o3d_mesh = convert_mesh(m)			
		geom = {'name': name, 'geometry': o3d_mesh, 'material': mat_target}
		geoms.append(geom)
	
	o3d.visualization.draw(geoms, title="LiDAR simulation")

def main():
	# arguments
	parser = argparse.ArgumentParser()
	# parser.add_argument('--input', default='model.obj', help='input mesh model file(.obj, .ply)')
	parser.add_argument('--input', default='model_complex1.obj', help='input mesh model file(.obj, .ply)')
	parser.add_argument('--output', default='output.pcd', help='output file(.pcd)')
	parser.add_argument('--pos', default='0.0,0.0,0.0', help='LiDAR position')
	parser.add_argument('--yaw', default=0.0, help='LiDAR yaw angle')
	parser.add_argument('--fov', default=60.0, help='LiDAR field of view')
	parser.add_argument('--interval', default=100, help='LiDAR interval count')
	parser.add_argument('--interval_angle', default=0.0, help='LiDAR interval angle')
	parser.add_argument('--range', default=10.0, help='LiDAR range')
	parser.add_argument('--noise', default=0.05, help='noise level')
	parser.add_argument('--multi_targets', default=0, help='multiple targets count')
	parser.add_argument('--viewer', default='on', help='run viewer = [on | off]')
	args = parser.parse_args()

	if args.interval_angle > 0.0:
		args.interval = int(args.fov / args.interval_angle)

	# create object
	mesh = None
	if len(args.input) > 0:	
		mesh = load_mesh(args.input)
	else:
		mesh = create_box(extents)
	if mesh is None:
		print('Failed to load mesh')
		return
	point_targets = []
	point_targets.append(mesh)
	if int(args.multi_targets) > 0:
		extents = mesh.extents	# extents = [5.0, 5.0, 3.0]
		point_targets = create_plane_targets(extents[1] / 2.0, args.multi_targets)

	# create virtual lidar and scan
	lidar = lidar_device()
	lidar.init(pos=[float(x) for x in args.pos.split(',')], yaw=args.yaw, fov_angle=args.fov, interval=args.interval, range=args.range, noise=args.noise)

	ray_origins, ray_directions = lidar.create_rays()
	ray_lines_set, intersection_points_set = lidar.scan(ray_origins, ray_directions, point_targets)

	# save intersection
	output_pcd = o3d.geometry.PointCloud()
	for index, intersection_points in enumerate(intersection_points_set):
		output_pcd += intersection_points
	o3d.io.write_point_cloud(args.output, output_pcd)
	print('output file = ', args.output)

	if args.viewer == 'on':
		view_simulation_results(ray_lines_set, intersection_points_set, point_targets, mesh)

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		traceback.print_exc()
