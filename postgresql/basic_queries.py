
insert_measurement = "INSERT INTO measurements(stamp, label, robot_position," \
                         " convex_hull, centroid_3d) VALUES (" \
                         "'{}', '{}', ST_MakePoint({}, {}, {}), ST_GeomFromEWKT('{}'), ST_Centroid('{}'));"


def retrieve_new_anchor_measurements(conn, cur):

    akey_list = retrieve_anchors(cur)
    measures = []
    for anchor_key, union_chull in akey_list:

        # Select measurements associated to anchor that have not already been used to compute the convex hull union
        q = "SELECT convex_hull, m.object_key "\
            "from measurements as m, measurements_anchors as ma, anchors as a "\
            "WHERE  a.anchor_key = '{}' AND m.object_key = ma.object_key " \
            "AND ma.anchor_key = a.anchor_key AND m.used_for_chull = FALSE"
        cur.execute(q.format(anchor_key))
        measures = [(r[0], r[1]) for r in cur.fetchall()] #, anchor_key, union_chull) for r in cur.fetchall()]

        # compute union of convex hulls in same anchor


    return measures

def retrieve_anchors(cur):

    # select all anchors marked as not complete (and their current union convex hull)
    q1 = "SELECT anchor_key, convex_hull_union FROM anchors" \
         " WHERE complete = FALSE"
    cur.execute(q1)
    return [(r[0], r[1]) for r in cur.fetchall()]


def update_union_chull(ma_list, connection, cursor):

    pass


