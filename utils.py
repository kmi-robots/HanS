import sys
import csv
csv.field_size_limit(sys.maxsize) # handle large file

from sensor_msgs_py import point_cloud2 as pc2
import numpy as np
import open3d as o3d
from collections import Counter
from boundingbox_msg.msg import ConvexHull
from geometry_msgs.msg import Polygon, Point32

import networkx as nx
import matplotlib.pyplot as plt


def get_csv_data(filepath, delim=','):
    with open(filepath, "rt") as csvfile:
        datareader = csv.reader(csvfile, delimiter=delim)
        for row in datareader: yield row

def load_camera_intrinsics_txt(path_to_intr):
    """
    Expects 3x3 intrinsics matrix as singlespace-separated txt
    See ./data/camera_intrinsics.txt for expected format
    """
    intrinsics = []
    with open(path_to_intr) as f:
        reader = csv.reader(f, delimiter=' ')
        for row in reader:
            if not row:
                continue
            for cell in row:
                if cell == '':
                    continue
                try:
                    intrinsics.append(float(cell.split("  ")[1]))
                except IndexError:
                    try:
                        intrinsics.append(float(cell.split(" ")[1]))
                    except IndexError:
                        intrinsics.append(float(cell))
    return intrinsics

def plot_graph(G):
    # G = nx.subgraph(G, ["0_radiator", "wall", "floor"]) #for debugging: look at node subset
    pos = nx.spring_layout(G)
    nx.draw(G, pos, connectionstyle='arc3, rad = 0.1', with_labels=True)
    edge_labels = nx.get_edge_attributes(G, 'QSR')
    # Uncomment below for debugging: look at only a subset of QSRs, e.g., leansOn, affixedON
    # edge_labels = {(u,v,i): d for (u,v,i), d in nx.get_edge_attributes(G, 'QSR').items() if d in ['leansOn','affixedOn']}
    draw_networkx_edge_labels(G, pos,
                              edge_labels=edge_labels)  # modified built-in method in nx below, because it needs unique keys, i.e., fails for multi-graph
    plt.draw()
    plt.show()


def draw_networkx_edge_labels(
        G,
        pos,
        edge_labels=None,
        label_pos=0.5,
        font_size=10,
        font_color="k",
        font_family="sans-serif",
        font_weight="normal",
        alpha=None,
        bbox=None,
        horizontalalignment="center",
        verticalalignment="center",
        ax=None,
        rotate=True,
):
    """Draw edge labels.
    Extended from original Networkx method to use with MultiDGraph,
    also refer to the official examples and docs at
    https://networkx.github.io/documentation/latest/auto_examples/index.html
    """

    if ax is None:
        ax = plt.gca()
    if edge_labels is None:
        labels = {(u, v): d for u, v, d in G.edges(data=True)}
    else:
        labels = edge_labels
    text_items = {}
    for (n1, n2, _), label in labels.items():  # only modification
        (x1, y1) = pos[n1]
        (x2, y2) = pos[n2]
        (x, y) = (
            x1 * label_pos + x2 * (1.0 - label_pos),
            y1 * label_pos + y2 * (1.0 - label_pos),
        )

        if rotate:
            # in degrees
            angle = np.arctan2(y2 - y1, x2 - x1) / (2.0 * np.pi) * 360
            # make label orientation "right-side-up"
            if angle > 90:
                angle -= 180
            if angle < -90:
                angle += 180
            # transform data coordinate angle to screen coordinate angle
            xy = np.array((x, y))
            trans_angle = ax.transData.transform_angles(
                np.array((angle,)), xy.reshape((1, 2))
            )[0]
        else:
            trans_angle = 0.0
        # use default box of white with white border
        if bbox is None:
            bbox = dict(boxstyle="round", ec=(1.0, 1.0, 1.0), fc=(1.0, 1.0, 1.0))
        if not isinstance(label, str):
            label = str(label)  # this makes "1" and 1 labeled the same

        t = ax.text(
            x,
            y,
            label,
            size=font_size,
            color=font_color,
            family=font_family,
            weight=font_weight,
            alpha=alpha,
            horizontalalignment=horizontalalignment,
            verticalalignment=verticalalignment,
            rotation=trans_angle,
            transform=ax.transData,
            bbox=bbox,
            zorder=1,
            clip_on=True,
        )
        text_items[(n1, n2)] = t

    ax.tick_params(
        axis="both",
        which="both",
        bottom=False,
        left=False,
        labelbottom=False,
        labelleft=False,
    )

    return text_items





def derive_object_cloud(bcoords, pmsg, mask=None):
    """Segment point cloud based on bounding box coords"""

    uvs = []

    if mask is None:
        for x in range(bcoords[0], bcoords[2]):
            for y in range(bcoords[1], bcoords[3]):
                uvs.append([x, y])
    else:
        for x in range(bcoords[0], bcoords[2]):
            for y in range(bcoords[1], bcoords[3]):
                if mask[y - bcoords[1]][x - bcoords[0]]:
                    uvs.append([x, y])

    cloud_data = list(pc2.read_points(pmsg, skip_nans=True, field_names=['x', 'y', 'z'], uvs=uvs))
    segm_cloud = o3d.geometry.PointCloud()
    xyz = [(x, y, z) for x, y, z in cloud_data]  # get xyz
    segm_cloud.points = o3d.utility.Vector3dVector(np.array(xyz))

    # o3d.visualization.draw_geometries([segm_cloud])
    return segm_cloud


def pcl_remove_outliers(obj_pcl, trans, params):
    # # Expects pointcloud in open3D format
    # uni_down_pcd = obj_pcl.uniform_down_sample(every_k_points=params.vx)
    # neighbours = int(len(np.asarray(uni_down_pcd.points)))
    # # print(neighbours)
    # fpcl, _ = obj_pcl.remove_statistical_outlier(nb_neighbors=neighbours, std_ratio=params.std_r)
    # # o3d.visualization.draw_geometries([fpcl])

    fpcl = obj_pcl.voxel_down_sample(voxel_size=0.02)

    labels = np.array(fpcl.cluster_dbscan(eps=params.eps, min_points=params.minp))
    if len(labels) < 1:
        print("No cluster found. Skipping")
        return fpcl

    cluster_size = Counter(labels)
    max_value = cluster_size[max(cluster_size, key=cluster_size.get)]
    main_clusters = {key: val for key, val in cluster_size.items() if val > max_value * 0.5}

    id_keep = []
    for count, label in enumerate(labels):
        if label in main_clusters:
            id_keep.append(count)

    clustered_pcl = fpcl.select_by_index(id_keep)
    # o3d.visualization.draw_geometries([clustered_pcl])
    clustered_pcl.transform(trans)
    return clustered_pcl


def derive_convex_hull(in_pcl, preds_, pmsg_, id_):
    hull, _ = in_pcl.compute_convex_hull()

    return hull2msg(hull, preds_, pmsg_, id_)


def hull2msg(convex_hull, predictions, pcloud_msg, hull_id):
    hull_message = ConvexHull()

    hull_message.id = hull_id
    hull_message.label = str(predictions)
    hull_message.header = pcloud_msg.header
    hull_message.header.frame_id = 'map'

    convex_hull.orient_triangles()

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

    return hull_message
