import re

insert_measurement = "INSERT INTO measurements(stamp, label, robot_position," \
                         " convex_hull, centroid_3d) VALUES (" \
                         "'{}', '{}', ST_MakePoint({}, {}, {}), ST_GeomFromEWKT('{}'), ST_GeometricMedian('{}'));"


def retrieve_new_anchor_measurements(conn, cur):

    akey_list = retrieve_anchors(cur)
    obj_anchors = {a_:{} for a_,_ in akey_list}
    for anchor_key, union_chull in akey_list:

        obj_anchors[anchor_key]['convex_hull'] = union_chull
        obj_anchors[anchor_key]['DL_predictions'] = []
        """"# Select measurements associated to anchor that have not already been used to compute the convex hull union
        q = "SELECT convex_hull, m.object_key "\
            "from measurements as m, measurements_anchors as ma, anchors as a "\
            "WHERE  a.anchor_key = '{}' AND m.object_key = ma.object_key " \
            "AND ma.anchor_key = a.anchor_key AND m.used_for_chull = FALSE"
        cur.execute(q.format(anchor_key))
        measures = [(r[0], r[1]) for r in cur.fetchall()] #, anchor_key, union_chull) for r in cur.fetchall()]"""

        # Select DL predictions of measurements associated to that anchor
        q = "SELECT m.label " \
            "from measurements as m, measurements_anchors as ma, anchors as a " \
            "WHERE  a.anchor_key = '{}' AND m.object_key = ma.object_key " \
            "AND ma.anchor_key = a.anchor_key"
        cur.execute(q.format(anchor_key))
        for r in cur.fetchall():
            pred_string = r[0]
            #parse string repr of list of tuples to list of tuples
            pred_list = [tuple(x.split(',')) for x in re.findall("\((.*?)\)", pred_string)]
            pred_list = [(int(cid), float(sc)) for cid, sc in pred_list]
            obj_anchors[anchor_key]['DL_predictions'].append(pred_list)

    return obj_anchors

def retrieve_anchors(cur):

    # select all anchors marked as not complete (and their current union convex hull)
    q1 = "SELECT anchor_key, convex_hull_union FROM anchors" \
         " WHERE complete = FALSE"
    cur.execute(q1)
    return [(r[0], r[1]) for r in cur.fetchall()]


