#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np
import cv2

from plane_detection_interfaces.srv import DetectPlanes, DetectPlanesResponse, DetectPlanesRequest
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from visualization_msgs.msg import MarkerArray, Marker
import tf2_ros


# Constants defining the frame and topic to subscribe to
FRAME = 'base_footprint'
POINTCLOUD_TOPIC = '/xtion/depth_registered/points'

class PlaneDetection:
    def __init__(self, frame=FRAME, pointcloud_topic=POINTCLOUD_TOPIC):
        self.frame = frame
        self.pointcloud_topic = pointcloud_topic

        # Initialize the ROS node
        rospy.init_node("plane_detection")
        
        # Initialize Transform Listener and Buffer for Cloud Transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize ROS Publishers
        self.pub = rospy.Publisher('/plane_detection/markers', MarkerArray, queue_size=10)

        # Initialize ROS Subscriber
        self.cloud_msg = None
        rospy.Subscriber(self.pointcloud_topic, PointCloud2, lambda msg: setattr(self, 'cloud_msg', msg))

        # Initialize ROS Service
        rospy.Service('/plane_detection/detect_planes', DetectPlanes, self.service_detect_planes)

        # Keep the node running
        rospy.spin()

    def service_detect_planes(self, req: DetectPlanesRequest) -> DetectPlanesResponse:
        # Set default values if not specified
        req.radius_search = req.radius_search if req.radius_search != 0.0 else 0.5
        req.max_nn = req.max_nn if req.max_nn != 0 else 30
        req.normal_variance_threshold_deg = req.normal_variance_threshold_deg if req.normal_variance_threshold_deg != 0.0 else 60.0
        req.coplanarity_deg = req.coplanarity_deg if req.coplanarity_deg != 0.0 else 75.0
        req.outlier_ratio = req.outlier_ratio if req.outlier_ratio != 0.0 else 0.75
        req.min_plane_edge_length = req.min_plane_edge_length if req.min_plane_edge_length != 0.0 else 0.25
        req.min_num_points = req.min_num_points if req.min_num_points != 0 else 0
        req.search_nn = req.search_nn if req.search_nn != 0 else 50
        req.distance_threshold = req.distance_threshold if req.distance_threshold != 0.0 else 0.01
        req.ransac_n = req.ransac_n if req.ransac_n != 0 else 3
        req.num_iterations = req.num_iterations if req.num_iterations != 0 else 1000

        response = DetectPlanesResponse()
        if self.cloud_msg is None:
            rospy.logwarn("No pointcloud received yet")
            return response
        
        # Transform the pointcloud to the base_footprint frame
        cloud_msg = self.transform_cloud(self.cloud_msg, self.frame)
        points = list(pc2.read_points(cloud_msg, skip_nans=True))
        
        # Only use spatial coordinates
        points = [p[:3] for p in points]

        # Create Open3D point cloud
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points)

        rospy.loginfo("Start plane segmentation with following parameters:\n"+str(req))

        # TODO: Implement plane segmentation
        return response

    def transform_cloud(self, cloud: PointCloud2, target_frame: str) -> PointCloud2:
        """
        This method transforms a ROS PointCloud2 message to a target frame using tf2.

        Parameters:
        cloud (PointCloud2): A ROS PointCloud2 message that will be transformed.
        target_frame (str): The target frame to which the point cloud will be transformed.

        Returns:
        PointCloud2: A transformed PointCloud2 message in the target frame.
        """
        trans = self.tf_buffer.lookup_transform(target_frame,
                                                cloud.header.frame_id,
                                                rospy.Time(0),
                                                rospy.Duration(1.0))
        return do_transform_cloud(cloud, trans)
    
if __name__ == '__main__':
    PlaneDetection()