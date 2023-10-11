import numpy as np
import open3d as o3d
from compas.geometry import oriented_bounding_box_numpy
from typing import List, Tuple, Union


def detect_plane(cloud: o3d.geometry.PointCloud, distance_threshold: float=0.02,
                 ransac_n: int=3, num_iterations: int=1000, horizontal_threshold: float=0.1,
                 plane_min_size: Tuple[float, float]=(0.3, 0.3)) -> Union[Tuple[List[float], o3d.geometry.PointCloud, List[np.ndarray], List[np.ndarray]], None]:
    """
    Detect the largest horizontal plane in an Open3D point cloud.

    Args:
    cloud (o3d.geometry.PointCloud): Input point cloud for plane detection.
    distance_threshold (float): Maximum distance from a point to the plane to be considered an inlier.
    ransac_n (int): The number of points randomly sampled to detect a plane.
    num_iterations (int): The maximum number of iterations for finding a plane.
    horizontal_threshold (float): Defines what 'horizontal' means. If the x and y parts of the normal vector are lower than this, the plane is considered as horizontal.
    plane_min_size (Tuple[float, float]): Minimum size of the plane to be considered valid.

    Returns:
    Union[Tuple[list, o3d.geometry.PointCloud, List[np.ndarray], List[np.ndarray]], None]: Parameters for plane equation [a, b, c, d], point cloud of all inliers, bounding box for the plane, and bounding box for the floor. Returns None if no plane found.
    """

    while len(cloud.points) >= ransac_n:  # While there are enough points to compute a plane
        plane_model, index = cloud.segment_plane(distance_threshold, ransac_n, num_iterations)
        inlier_cloud = cloud.select_by_index(index)
        
        # Remove detected plane from cloud
        cloud = cloud.select_by_index(index, invert=True)
        [a, b, c, d] = plane_model

        # The normal vector should point in the z axis; otherwise, it is not a horizontal plane
        if abs(a) > horizontal_threshold or abs(b) > horizontal_threshold:
            continue

        # Cluster all points inside the plane
        clusters, _ = db_clustering(inlier_cloud, eps=0.04, min_points=100)

        # Compute Bounding Boxes for each cluster
        # Return the first cluster where the scale make sense to be a table plane
        for cluster in clusters:
            bb_floor = compute_bb_on_plane(cluster.points, [0.0, 0.0, 1.0, 0.0])
            bb = oriented_bounding_box_numpy(cluster.points)
            # Compute the scale of the box in each direction
            scale_x = np.linalg.norm(bb[1] - bb[0])
            scale_y = np.linalg.norm(bb[3] - bb[0])
            #scale_z = np.linalg.norm(bb[4] - bb[0])

            if scale_x > plane_min_size[0] and scale_y > plane_min_size[1]:
                return plane_model, cluster, bb, bb_floor

    # Raise exception if no plane could be found
    raise Exception('No plane found')



def db_clustering(cloud: o3d.geometry.PointCloud, eps: float=0.05, min_points: int=20) -> Tuple[List[o3d.geometry.PointCloud], o3d.geometry.PointCloud]:
    """
    Cluster a point cloud using Open3D's DBSCAN method. Returns clusters and noise points.

    Args:
    cloud (o3d.geometry.PointCloud): Input point cloud for clustering.
    eps (float): The maximum distance between two samples for one to be considered as in the neighborhood of the other.
    min_points (int): The number of samples in a neighborhood for a point to be considered as a core point.

    Returns:
    Tuple[List[o3d.geometry.PointCloud], o3d.geometry.PointCloud]: List of point clouds representing each cluster and a point cloud of noise points.
    """
    clusters = []

    # Perform DBSCAN clustering
    labels = np.array(cloud.cluster_dbscan(eps=eps, min_points=min_points))

    # Select noise points (label == -1)
    noise = cloud.select_by_index(list(np.where(labels == -1)[0]))
    
    # Create a point cloud for each cluster
    for i in range(labels.max()+1):
        cluster = cloud.select_by_index(list(np.where(labels == i)[0]))
        clusters.append(cluster)
    
    return clusters, noise



def filter_points_below_plane(points: List[np.ndarray], plane_model: List[float], threshold: float=0.02) -> List[np.ndarray]:
    """
    Filter out points that are below a given plane.

    Args:
    points (List[np.ndarray]): List of 3D points.
    plane_model (List[float]): Parameters of the plane equation in the form [a, b, c, d].
    threshold (float): Used to offset the plane in the normal direction.

    Returns:
    List[np.ndarray]: List of points that are above the plane.
    """
    (a, b, c, d) = plane_model

    # Select points above the plane
    points_above_plane = [p for p in points
                          if a*p[0] + b*p[1] + c*p[2] + d > threshold]
    return points_above_plane


def compute_bb_on_plane(points: np.ndarray, plane_model: List[float]) -> np.ndarray:
    """
    Project the points onto the plane and compute an oriented bounding box.

    Args:
    points (np.ndarray): Array of points.
    plane_model (List[float]): The model parameters of the plane [a, b, c, d].

    Returns:
    np.ndarray: Oriented bounding box for the projected points.
    """
    projected_points = project_points_onto_plane(points, plane_model)
    return oriented_bounding_box_numpy(np.concatenate((np.asarray(projected_points), np.asarray(points))))

def project_points_onto_plane(points: np.ndarray, plane_eq: List[float]) -> List[np.ndarray]:
    """
    Projects a set of points onto a plane.

    Args:
    points (np.ndarray): A numpy array of points to be projected.
    plane_eq (List[float]): A list of four floats defining the plane equation ax + by + cz + d = 0.

    Returns:
    List[np.ndarray]: A list of numpy arrays representing the points projected onto the plane.
    """
    a, b, c, d = plane_eq

    # Normalize the plane equation
    magnitude = np.sqrt(a**2 + b**2 + c**2)
    a /= magnitude
    b /= magnitude
    c /= magnitude
    d /= magnitude

    # Compute the projection of each point onto the plane
    projected_points = []
    for point in points:
        x, y, z = point

        # Compute the distance from the point to the plane
        distance = a * x + b * y + c * z + d

        # Project the point onto the plane
        projected_point = np.array([x - a * distance, y - b * distance, z - c * distance])
        projected_points.append(projected_point)

    return projected_points