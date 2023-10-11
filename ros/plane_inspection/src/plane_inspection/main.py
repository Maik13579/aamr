#!/usr/bin/env python3

import rospy
import open3d as o3d
import numpy as np

from plane_inspection_interfaces.srv import InspectPlane, InspectPlaneResponse, InspectPlaneRequest
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud
from geometry_msgs.msg import PoseStamped
from tf2_geometry_msgs.tf2_geometry_msgs import do_transform_pose
from visualization_msgs.msg import MarkerArray
import tf2_ros
from visualization_msgs.msg import Marker
from typing import List, Dict, Tuple

from plane_inspection.conversion import create_marker_from_bb
from plane_inspection.pointcloud_utils import detect_plane, filter_points_below_plane, db_clustering, compute_bb_on_plane

# Constants defining the frame and topic to subscribe to
FRAME = 'base_footprint'
POINTCLOUD_TOPIC = '/xtion/depth_registered/points'


class PlaneInspection:
    """
    This class performs a Plane Inspection on a 3D Point Cloud utilizing ROS and Open3D.
    The class includes ROS publishers, subscribers, and a service that detects and publishes the largest
    horizontal plane in a point cloud, along with bounding boxes around clusters.
    """
    def __init__(self, pointcloud_topic: str = POINTCLOUD_TOPIC):
        """
        The constructor for the PlaneInspection class initializes the ROS node, Transform Listener and Buffer for 
        Cloud Transformation, ROS publishers, ROS subscribers and ROS service. Finally, it starts the ROS node.
        """
        
        # Initialize the ROS node
        rospy.init_node("plane_inspection")
        
        # Initialize Transform Listener and Buffer for Cloud Transformation
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Initialize ROS Publishers
        self.pub = rospy.Publisher('/plane_inspection/markers', MarkerArray, queue_size=10)

        # Initialize ROS Subscriber
        self.cloud_msg = None
        rospy.Subscriber(pointcloud_topic, PointCloud2, lambda msg: setattr(self, 'cloud_msg', msg))

        # Initialize ROS Service
        rospy.Service('/plane_inspection/inspect_plane', InspectPlane, self.service_inspect_plane)

        # Keep the node running
        rospy.spin()

    def service_inspect_plane(self, req: InspectPlaneRequest) -> InspectPlaneResponse:
        """
        The callback function for the inspect_plane service.
        This function performs plane detection, point filtering, clustering, and bounding box calculation. 

        Parameters:
        req (InspectPlaneRequest): The request message which includes the following parameters: base_footprint_frame, 
                                min_height, max_height, distance_threshold, ransac_n, num_iterations, horizontal_threshold,
                                plane_min_size_x, plane_min_size_y, eps, and min_points.

        Returns:
        InspectPlaneResponse: The response message which includes the detected plane, the plane's equation coefficients, 
                            and bounding boxes for objects.
        """

        # Wait for a point cloud to be available
        while not self.cloud_msg:
            rospy.sleep(0.1)

        # Initialize request parameters with provided values or defaults
        base_footprint = req.base_footprint_frame if req.base_footprint_frame != '' else FRAME
        min_height = req.min_height if req.min_height != 0.0 else 0.3
        max_height = req.max_height if req.max_height != 0.0 else 2.0
        distance_threshold = req.distance_threshold if req.distance_threshold != 0.0 else 0.01
        ransac_n = req.ransac_n if req.ransac_n != 0 else 3
        num_iterations = req.num_iterations if req.num_iterations != 0 else 1000
        horizontal_threshold = req.horizontal_threshold if req.horizontal_threshold != 0.0 else 0.1
        plane_min_size_x = req.plane_min_size_x if req.plane_min_size_x != 0.0 else 0.1
        plane_min_size_y = req.plane_min_size_y if req.plane_min_size_y != 0.0 else 0.1
        plane_min_size = (plane_min_size_x, plane_min_size_y)
        eps = req.eps if req.eps != 0.0 else 0.025
        min_points = req.min_points if req.min_points != 0 else 50

        res = InspectPlaneResponse()
        marker_array = MarkerArray()

        # Get the point cloud data
        scan = self.get_scan_cloud(base_footprint, min_height, max_height)
        
        # Detect the largest horizontal plane
        rospy.loginfo('Detect plane:')
        plane_model, _, plane_bb, plane_bb_floor = detect_plane(scan, distance_threshold,
                                                  ransac_n, num_iterations,
                                                  horizontal_threshold,
                                                  plane_min_size)
        a, b, c, d = plane_model
        plane_marker = create_marker_from_bb(plane_bb, 'plane', base_footprint, color=(1.0, 0.0, 0.0))
        plane_marker_floor = create_marker_from_bb(plane_bb_floor, 'plane_floor', base_footprint, color=(1.0, 0.0, 0.0))

        res.plane = plane_marker
        res.plane_floor = plane_marker_floor
        marker_array.markers.append(plane_marker)
        marker_array.markers.append(plane_marker_floor)
        res.a = a
        res.b = b
        res.c = c
        res.d = d
        rospy.loginfo(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")

        # Filter points below the plane
        rospy.loginfo('Filter points below plane')
        points_above_plane = filter_points_below_plane(scan.points, plane_model, threshold=0.01)

        # Create Open3D point cloud from the points above the plane
        above_plane = o3d.geometry.PointCloud()
        above_plane.points = o3d.utility.Vector3dVector(points_above_plane)

        # Cluster the points above the plane
        rospy.loginfo("Cluster points above the plane")
        clusters, _ = db_clustering(above_plane, eps, min_points)
        rospy.loginfo(str(len(clusters))+" clusters detected")

        # Compute bounding boxes for each cluster
        rospy.loginfo("Compute bounding boxes for each cluster")
        bbs = compute_bbs_from_clusters(clusters, plane_model, base_footprint, plane_bb)
        rospy.loginfo(str(len(bbs))+' clusters are ontop of the plane.')

        for _, marker in bbs.items():
            marker_array.markers.append(marker)
            res.objects_bb.append(marker)

        self.pub.publish(marker_array)
        rospy.loginfo('#'*50)
        rospy.loginfo('Final Result:')
        rospy.loginfo(f"Plane equation: {a:.2f}x + {b:.2f}y + {c:.2f}z + {d:.2f} = 0")
        rospy.loginfo('Objects: '+str(len(res.objects_bb)))
        return res

    def get_scan_cloud(self, target_frame: str, min_height: float=0.3, max_height:float=2.0) -> o3d.geometry.PointCloud:
        """
        This method transforms the point cloud to the target frame, filters points that are not within the defined height 
        range (min_height to max_height), and creates an Open3D PointCloud object.

        Parameters:
        target_frame (str): The target frame to which the point cloud will be transformed.
        min_height (float, optional): The minimum height of points. Points below this height will be filtered out. Defaults to 0.3.
        max_height (float, optional): The maximum height of points. Points above this height will be filtered out. Defaults to 2.0.

        Returns:
        o3d.geometry.PointCloud: An Open3D PointCloud object with points within the specified height range and transformed to the target frame.
        """

        # Transform the point cloud to target frame
        scan_msg = self.transform_cloud(self.cloud_msg, target_frame)

        points = list(pc2.read_points(scan_msg, skip_nans=True))

        # Remove points that don't lie within the defined height range
        points = [p[:3] for p in points if p[2] >= min_height and p[2] <= max_height]

        # Create Open3D point cloud
        o3d_cloud = o3d.geometry.PointCloud()
        o3d_cloud.points = o3d.utility.Vector3dVector(points)
        return o3d_cloud
    
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
    
    def transform_pose(self, pose: PoseStamped, target_frame: str) -> PoseStamped:
        """
        This method transforms a ROS PoseStamped message to a target frame using tf2.

        Parameters:
        pose (PoseStamped): A ROS PoseStamped message that will be transformed.
        target_frame (str): The target frame to which the pose will be transformed.

        Returns:
        PoseStamped: A transformed PoseStamped message in the target frame.
        """

        trans = self.tf_buffer.lookup_transform(target_frame,
                                                pose.header.frame_id,
                                                rospy.Time(0),
                                                rospy.Duration(1.0))
        return do_transform_pose(pose, trans)
    
def compute_bbs_from_clusters(clusters: List[o3d.geometry.PointCloud], plane_model: Tuple[float, float, float, float], frame: str, plane_bb: List[List[float]]) -> Dict[int, Marker]:
    """
    Compute bounding boxes for all clusters, convert them to marker arrays and check if they are on top of the plane.
    
    Parameters:
    clusters (List[PointCloud]): A list of clusters, where each cluster is represented as an Open3D PointCloud object.
    plane_model (Tuple[float, float, float, float]): The parameters (a, b, c, d) of the plane equation ax + by + cz + d = 0.
    frame (str): The target frame that the bounding boxes will be referred to.
    plane_bb (List[List[float]]): The corners of the bounding box for the plane, as a list of [x, y, z] coordinate lists.

    Returns:
    bbs (Dict[int, Marker]): A dictionary mapping from marker IDs to the corresponding Marker objects for bounding boxes.
    """
    bbs = {}
    plane_x = [corner[0] for corner in plane_bb]
    plane_y = [corner[1] for corner in plane_bb]
    for cluster in clusters:
            bb = compute_bb_on_plane(cluster.points, plane_model)
            marker = create_marker_from_bb(bb, 'unkown_objects', frame, color=(0.0, 0.0, 1.0))

            #check if bb is ontop of plane
            if marker.pose.position.x < min(plane_x) \
                or marker.pose.position.x > max(plane_x) \
                or marker.pose.position.y < min(plane_y) \
                or marker.pose.position.y > max(plane_y):
               continue

            bbs[marker.id] = marker
    return bbs

if __name__ == '__main__':
    # Instantiate the PlaneInspection class
    import sys

    if len(sys.argv) > 1:
        PlaneInspection(sys.argv[1])
    elif len(sys.argv) > 2:
        PlaneInspection(sys.argv[1], sys.argv[2])
    elif len(sys.argv) > 3:
        FRAME = sys.argv[3]
        PlaneInspection(sys.argv[1], sys.argv[2])
    else:
        PlaneInspection()
