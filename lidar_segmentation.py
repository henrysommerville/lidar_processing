#!/usr/bin/env python3

import rospy
from std_msgs.msg import Header
import numpy as np
import struct
import numpy as np
from sensor_msgs.msg import PointCloud2, PointField
from sensor_msgs import point_cloud2

<<<<<<< HEAD
#comment
=======

>>>>>>> origin/main
class LidarProcessor:
    def __init__(self):
        # Initialize ROS node
        rospy.init_node('lidar_processor')

        # Create subscribers and publishers
        rospy.Subscriber('/livox/lidar', PointCloud2, self.lidar_callback)
        self.ground_publisher = rospy.Publisher('/ground', PointCloud2, queue_size=1)
        self.nonground_publisher = rospy.Publisher('/nonground', PointCloud2, queue_size=1)

        # Initialize RANSAC algorithm parameters
        self.max_distance = 0.2
        self.num_iterations = 75

    def lidar_callback(self, msg):
        # Extract point cloud data as numpy array
        point_cloud = np.array([[point[0], point[1], point[2]] for point in point_cloud2.read_points(msg)])

        # Segment point cloud into ground and non-ground points
        ground_points, nonground_points = self.segment_lidar_data(point_cloud)

        # Publish ground and non-ground points as separate PointCloud2 messages
        self.publish_points(self.ground_publisher, ground_points, msg.header)
        self.publish_points(self.nonground_publisher, nonground_points, msg.header)

    def segment_lidar_data(self, point_cloud):
        # Segment ground points using RANSAC algorithm
        # Assume that ground is the largest plane in the scene
        plane_model, inliers = self.ransac_segmentation(point_cloud)

        # Extract ground points
        ground_points = point_cloud[inliers]

        # Extract non-ground points
        non_ground_points = point_cloud[np.logical_not(inliers)]

        return ground_points.tolist(), non_ground_points.tolist()

    def ransac_segmentation(self, points):
        best_model = None
        best_inliers = None
        best_num_inliers = 0

        for i in range(self.num_iterations):
            # Randomly select 3 points
            indices = np.random.choice(points.shape[0], 3, replace=False)
            sample_points = points[indices]

            # Compute plane model using selected points
            normal = np.cross(sample_points[1] - sample_points[0], sample_points[2] - sample_points[0])
            normal /= np.linalg.norm(normal)
            d = -np.dot(normal, sample_points[0])
            plane_model = np.hstack((normal, d))

            # Compute distance between all points and plane model
            distances = np.abs(np.dot(points, plane_model[:3]) + plane_model[3])
            inliers = distances < self.max_distance
            num_inliers = np.count_nonzero(inliers)

            # Update best model if current model has more inliers
            if num_inliers > best_num_inliers:
                best_model = plane_model
                best_inliers = inliers
                best_num_inliers = num_inliers

        return best_model, best_inliers
    

    def publish_points(self, publisher, points, header):
        # Create point cloud message
        point_cloud = PointCloud2()
        point_cloud.header = header

        # Set point cloud fields
        fields = [PointField('x', 0, PointField.FLOAT32, 1),
                PointField('y', 4, PointField.FLOAT32, 1),
                PointField('z', 8, PointField.FLOAT32, 1)]
        point_cloud.fields = fields
        point_cloud.is_bigendian = False
        point_cloud.point_step = 12
        point_cloud.row_step = point_cloud.point_step * len(points)
        point_cloud.height = 1
        point_cloud.width = len(points)
        point_cloud.is_dense = True

        # Convert points to binary data and set point cloud data
        point_data = np.array(points, dtype=np.float32)
        point_data = point_data.flatten()
        point_cloud.data = struct.pack('%sf' % len(point_data), *point_data)

        # Publish point cloud message
        publisher.publish(point_cloud)
    
if __name__ == '__main__':
    lidar_segmentation = LidarProcessor()
    rospy.spin()