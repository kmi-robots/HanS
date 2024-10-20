"""Queries to estimate object dimensions based on oriented boxes"""
import math

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

def retrieve_dims(cur, anchor_k):
    """Given anchor key, returns list of estimated dimensions"""
    query = "SELECT d1, d2, d3 " \
            "FROM anchors " \
            "WHERE anchor_key = '{}' "

    cur.execute(query.format(anchor_k))
    return list(cur.fetchone())

def find_frontbox_shape(cur, anchor_k, dims):
    """Returns the height and width of the front halfspace
    that is used to then to derive the Aspect Ratio of the object"""
    # d3 is always the height, based on how we add measures (see populate_with_sizes method)
    query = "SELECT d3, frontd1, frontd2 " \
            "FROM anchors " \
            "WHERE anchor_key = '{}' "
    cur.execute(query.format(anchor_k))

    height, frontd1, frontd2 =list(cur.fetchone())
    # Compare frontw, frontd with object's d1 and d2 (that are inside dims)
    # The one between frontw and frontd that is equal to either of d1 and d2 is the dim we want

    check1= [math.isclose(frontd1, d) for d in dims]#almost equality of floats, could differ only by an epsilon at the nth decimal place
    check2= [math.isclose(frontd2, d) for d in dims]

    if any(check1):
        tgt_dim = frontd1
    elif any(check2):
        tgt_dim = frontd2

    return (height, tgt_dim)

