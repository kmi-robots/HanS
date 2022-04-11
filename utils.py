import csv
from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import open3d as o3d
from collections import Counter
# from boundingbox_msg.msg import ConvexHull
from geometry_msgs.msg import Polygon, Point32

def load_camera_intrinsics_txt(path_to_intr):
    """
    Expects 3x3 intrinsics matrix as singlespace-separated txt
    See ./data/camera_intrinsics.txt for expected format
    """
    intrinsics = []
    with open(path_to_intr) as f:
        reader = csv.reader(f, delimiter=' ')
        for row in reader:
            if row==[]: continue
            for cell in row:
                if cell=='': continue
                try:
                    intrinsics.append(float(cell.split("  ")[1]))
                except IndexError:
                    try:
                        intrinsics.append(float(cell.split(" ")[1]))
                    except IndexError:
                        intrinsics.append(float(cell))
    return intrinsics


def derive_object_cloud(bcoords, pmsg):
    """Segment point cloud based on bounding box coords"""

    uvs = []

    for x in range(bcoords[0], bcoords[2]):
        for y in range(bcoords[1], bcoords[3]):
            uvs.append([x, y])

    cloud_data = list(pc2.read_points(pmsg, skip_nans=True, field_names=['x', 'y', 'z'], uvs=uvs))
    segm_cloud = o3d.geometry.PointCloud()
    xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
    segm_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    return segm_cloud

def remove_outliers(obj_pcl,params):
    #Expects pointcloud in open3D format

    # Filter by distance
    dists = obj_pcl.compute_point_cloud_distance(obj_pcl)
    dists = np.asarray(dists)
    ind = np.where(dists > params.dist)[0]
    dist_filtered = obj_pcl.select_by_index(ind)

    #Density-based clustering
    labels = np.array(dist_filtered.cluster_dbscan(eps=params.eps, min_points=params.minp))
    if len(labels) < 1:
        print("No cluster found. Skipping")
        return dist_filtered
    print(f"point cloud has {labels.max() + 1} clusters")

    cluster_size = Counter(labels)
    max_value = cluster_size[max(cluster_size, key=cluster_size.get)]
    main_clusters = {key: val for key, val in cluster_size.items() if val > max_value * 0.5}

    id_keep = []
    for count, label in enumerate(labels):
        if label in main_clusters:
            id_keep.append(count)

    return dist_filtered.select_by_index(id_keep)

def derive_convex_hull(in_pcl, preds_, pmsg_):

    hull, _ = in_pcl.compute_convex_hull()

    return hull2msg(hull, preds_, pmsg_)


def hull2msg(convex_hull, predictions, pcloud_msg):

    hull_message = dict() #ConvexHull()

    hull_message.id = predictions[0] #TODO replace with timestamp?
    hull_message.label = predictions[1]
    hull_message.header = pcloud_msg.header
    hull_message.header.frame_id = 'map'

    for tr in convex_hull.triangles:
        poly = Polygon()
        for idv in tr:
            v = convex_hull.vertices[idv]
            ppoint = Point32()
            ppoint.x = v[0]
            ppoint.y = v[1]
            ppoint.z = v[2]
            poly.points.append(ppoint)
        v = convex_hull.vertices[tr[0]]
        ppoint = Point32()
        ppoint.x = v[0]
        ppoint.y = v[1]
        ppoint.z = v[2]
        poly.points.append(ppoint)
        hull_message.polygons.append(poly)


