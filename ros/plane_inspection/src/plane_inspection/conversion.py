import rospy

from std_msgs.msg import Header
from sensor_msgs.msg import PointField, PointCloud2
import sensor_msgs.point_cloud2 as pc2
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

from random import randrange

import numpy as np
from scipy.spatial.transform import Rotation
from typing import List, Tuple

def create_pc2_msg(points: List[np.ndarray], frame_id: str) -> PointCloud2:
    """
    Create a ROS sensor_msgs/PointCloud2 message.

    Args:
    points (List[np.ndarray]): List of 3D points.
    frame_id (str): The name of the coordinate frame the points are associated with.

    Returns:
    sensor_msgs/PointCloud2: Point cloud message with provided points and frame_id.
    """
    # Initializing the header with the provided frame_id and current timestamp
    header = Header()
    header.frame_id = frame_id
    header.stamp = rospy.Time.now()

    # Setting the structure of point fields in the PointCloud2 message
    fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    
    # Creating the PointCloud2 message
    return pc2.create_cloud(header, fields, points)



def create_marker_from_bb(bb: List[np.ndarray], ns: str, frame_id: str, color: Tuple[float, float, float]=(0.0, 0.0, 1.0)) -> Marker:
    """
    Create a visualization_msgs/Marker message from the 8 corner points of a bounding box.

    Args:
    bb (List[np.ndarray]): List of 8 corner points of the bounding box.
    ns (str): Namespace for the marker message.
    frame_id (str): The name of the coordinate frame the bounding box is associated with.

    Returns:
    visualization_msgs/Marker: Marker message representing the bounding box.
    """
    # Initializing the marker
    marker = Marker()
    marker.id = randrange(2**16)
    marker.ns = ns
    marker.type = marker.CUBE
    marker.header.frame_id = frame_id
    marker.color.a = 0.5
    marker.color.r = color[0]
    marker.color.g = color[1]
    marker.color.b = color[2]

    #Add corner points of bb
    for corner in bb:
        p = Point()
        p.x = corner[0]
        p.y = corner[1]
        p.z = corner[2]
        marker.points.append(p)

    # Extracting the bounding box properties
    center, scale, q = box_properties(bb)

    # Setting the position, scale, and orientation of the marker based on the bounding box properties
    marker.pose.position.x = center[0]
    marker.pose.position.y = center[1]
    marker.pose.position.z = center[2]
    marker.scale.x = scale[0]
    marker.scale.y = scale[1]
    marker.scale.z = scale[2]
    marker.pose.orientation.x = q[0]
    marker.pose.orientation.y = q[1]
    marker.pose.orientation.z = q[2]
    marker.pose.orientation.w = q[3]

    return marker


def box_properties(points: List[np.ndarray]) -> Tuple[np.ndarray, Tuple[float, float, float], np.ndarray]:
    """
    Compute center, scale, and orientation (as quaternion) from the 8 corners of a box.

    Args:
    points (List[np.ndarray]): List of 8 corner points of the bounding box.

    Returns:
    Tuple[np.ndarray, Tuple[float, float, float], np.ndarray]: The center, scale, and orientation of the box.
    """
    # Ensure the input is a numpy array
    points = np.array(points)

    # Compute the center of the box
    center = np.mean(points, axis=0)

    # Compute the scale of the box in each direction
    scale_x = np.linalg.norm(points[1] - points[0])
    scale_y = np.linalg.norm(points[3] - points[0])
    scale_z = np.linalg.norm(points[4] - points[0])

    # Compute the orientation of the box
    u1 = (points[1] - points[0]) / scale_x
    u2 = (points[3] - points[0]) / scale_y
    u3 = (points[4] - points[0]) / scale_z

    # Construct the rotation matrix
    rot_matrix = np.array([u1, u2, u3]).T

    # Convert the rotation matrix to a quaternion
    rot = Rotation.from_matrix(rot_matrix)
    quaternion = rot.as_quat()

    return center, (scale_x, scale_y, scale_z), quaternion