# title: simulate LiDAR from RGBD dataset
# author: taewook kang
# email: laputa99999@gmail.com
# version: draft. testing. 
# todo: load RGBD dataset, Etc.
# 
import trimesh, traceback
import numpy as np
from tqdm import tqdm
import open3d as o3d

def main():
	# test on a sphere mesh
	# mesh = trimesh.primitives.Sphere(radius = 1.0)
	mesh = trimesh.primitives.Box(extents = [5.0, 5.0, 3.0])
	
	point_targets = []
	for i in range(50):
		for j in range(50):
			step = 0.1
			x = i * step - 2.5
			y = 2.5
			z = j * step - 2.5
			target = trimesh.primitives.Box(extents = [0.1, 0.1, 0.1], transform = trimesh.transformations.translation_matrix([x, y, z]))
			# target = trimesh.primitives.Plane() TBD
			point_targets.append(target)

	# Define LiDAR parameters
	lidar_position = [0.0, 0.0, 0.0]  # LiDAR's position in the room
	lidar_yaw = 0.0  # LiDAR's yaw angle (rotation around the vertical axis)
	lidar_fov = 60.0 # 10 # 60  # LiDAR's field of view in degrees
	lidar_range = 10.0  # Maximum LiDAR range

	# create some rays
	ray_origins = None # np.array([[0.0, 0.5, 0.0]])
	ray_directions = None # np.array([[0.0, 0.5, 2.0]])
	
	# Generate LiDAR point cloud
	lidar_pcd = o3d.geometry.PointCloud()
	for azimuth in np.linspace(lidar_yaw - lidar_fov / 2, lidar_yaw + lidar_fov / 2, 10): #360):
		for elevation in np.linspace(-lidar_fov / 2, lidar_fov / 2, 10): # 180):
			# Compute laser beam direction in LiDAR's local coordinates
			direction = np.array([
				np.cos(np.radians(elevation)) * np.sin(np.radians(azimuth)),
				np.cos(np.radians(elevation)) * np.cos(np.radians(azimuth)),
				np.sin(np.radians(elevation)),
			])

			# Compute the end point of the laser beam
			end_point = np.array([
				lidar_position[0] + direction[0] * lidar_range,
				lidar_position[1] + direction[1] * lidar_range,
				lidar_position[2] + direction[2] * lidar_range,
			])

			if ray_origins is None:
				ray_origins = np.array([lidar_position])
				ray_directions = np.array([end_point - lidar_position])
			else:
				ray_origins = np.append(ray_origins, lidar_position)
				ray_directions = np.append(ray_directions, end_point - lidar_position)
	
	ray_origins = ray_origins.reshape(-1, 3)
	ray_directions = ray_directions.reshape(-1, 3)
	
	# run the mesh-ray test
	ray_lines_set = []
	intersection_points_set = []
	for target in tqdm(point_targets):
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
		intersection_points.points = o3d.utility.Vector3dVector(locations)
		intersection_points_set.append(intersection_points)

		# Create an Open3D LineSet to visualize rays
		ray_lines = o3d.geometry.LineSet()
		ray_lines.points = o3d.utility.Vector3dVector(np.vstack((ray_origins, ray_origins + ray_directions)))
		num_points = len(ray_origins)
		line_indices = np.array([[i, i + num_points] for i in range(num_points)])
		ray_lines.lines = o3d.utility.Vector2iVector(line_indices)    # Create lines between pairs of points. index. 0,1 is the first line, 2,3 is the second line
		ray_lines_set.append(ray_lines)

	# Create an Open3D TriangleMesh from the Trimesh mesh
	vertices = mesh.vertices.copy()  # Make a copy of the vertices array
	faces = mesh.faces.copy()        # Make a copy of the faces array
	o3d_mesh = o3d.geometry.TriangleMesh()
	o3d_mesh.vertices = o3d.utility.Vector3dVector(vertices)
	o3d_mesh.triangles = o3d.utility.Vector3iVector(faces)

	# Set visualization properties
	# o3d_mesh.paint_uniform_color([0.7, 0.7, 0.7])  # Set mesh color
	# intersection_points.paint_uniform_color([1, 0, 0])  # Set intersection points color
	# ray_lines.colors = o3d.utility.Vector3dVector(np.tile(np.array([0, 0, 1]), (2, 1)))  # Set ray color

	# Create an Open3D visualization window
	mat_point = o3d.visualization.rendering.MaterialRecord()
	mat_point.shader = 'defaultLit'
	mat_point.base_color = [1.0, 0, 0, 1.0]
	mat_point.thickness = 2.0
 
	mat_line = o3d.visualization.rendering.MaterialRecord()
	mat_line.shader = 'defaultLit'
	mat_line.base_color = [0.0, 0, 1.0, 1.0]

	mat_tar = o3d.visualization.rendering.MaterialRecord()
	mat_tar.shader = 'defaultLit'
	mat_tar.base_color = [1.0, 0, 1.0, 1.0]
	
	mat_box = o3d.visualization.rendering.MaterialRecord()
	# mat_box.shader = 'defaultLitTransparency'
	mat_box.shader = 'defaultLitSSR'
	mat_box.base_color = [0.467, 0.467, 0.467, 0.2]
	mat_box.base_roughness = 0.0
	mat_box.base_reflectance = 0.0
	mat_box.base_clearcoat = 1.0
	mat_box.thickness = 1.0
	mat_box.transmission = 0.5
	mat_box.absorption_distance = 10
	mat_box.absorption_color = [0.5, 0.5, 0.5]

	geoms = [{'name': 'box', 'geometry': o3d_mesh, 'material': mat_box}]
	
	for index, ray_lines in enumerate(ray_lines_set):
		name = 'line' + str(index)
		
		ray = {'name': name, 'geometry': ray_lines, 'material': mat_line}
		geoms.append(ray)
	
	for index, intersection_points in enumerate(intersection_points_set):
		name = 'intpoints' + str(index)
		
		pt = {'name': name, 'geometry': intersection_points, 'material': mat_point}
		geoms.append(pt)
 
	'''
	for index, m in enumerate(point_targets):
		name = 'box' + str(index)
		
		vertices = m.vertices.copy()  # Make a copy of the vertices array
		faces = m.faces.copy()        # Make a copy of the faces array
		o3d_mesh = o3d.geometry.TriangleMesh()
		o3d_mesh.vertices = o3d.utility.Vector3dVector(vertices)
		o3d_mesh.triangles = o3d.utility.Vector3iVector(faces)
			
		geom = {'name': name, 'geometry': o3d_mesh, 'material': mat_tar}
		geoms.append(geom)
	'''
 
	o3d.visualization.draw(geoms) # , window_name="LiDAR simulation")

	# o3d.visualization.draw_geometries([o3d_mesh, intersection_points, ray_lines])

if __name__ == '__main__':
	try:
		main()
	except Exception as e:
		traceback.print_exc()
