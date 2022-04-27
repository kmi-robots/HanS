"""Queries to estimate object dimensions based on oriented boxes"""

def populate_with_sizes(connection,cursor):

    # for all object anchors
    # Find min oriented 3D box bounding of polyhedral surface
    cursor.execute('SELECT anchor_key, ST_ZMin(convex_hull_union),'
                   ' ST_ZMax(convex_hull_union) FROM anchors')

    query_res = [(str(r[0]), float(r[1]), float(r[2])) for r in cursor.fetchall()]

    for id_, zmin, zmax in query_res:
        d3 = zmax - zmin  #height based on 3D bbox

        # Edges of 2D oriented envelope give remaining two dims
        dquery = 'SELECT  ST_Distance(ST_PointN(ST_ExteriorRing(oriented_envelope), 1), ' \
                'ST_PointN(ST_ExteriorRing(oriented_envelope), 2)) AS d1, ' \
                'ST_Distance(ST_PointN(ST_ExteriorRing(oriented_envelope), 2) ,' \
                'ST_PointN(ST_ExteriorRing(oriented_envelope), 3)) AS d2 ' \
                'FROM anchors ' \
                 'WHERE anchor_key = %s;'

        cursor.execute(dquery,(id_,))
        d1,d2 = cursor.fetchone()
        sizeq= 'UPDATE anchors SET d1 = %s, d2= %s, d3=%s '\
                'WHERE anchor_key = %s;'
        cursor.execute(sizeq, (d1,d2,d3,id_))
        connection.commit()
