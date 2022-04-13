
insert_measurement = "INSERT INTO measurements(stamp, tracking_id, label," \
                         " convex_hull, centroid_3d) VALUES (" \
                         "'{}', '{}', '{}', ST_GeomFromEWKT('{}'), ST_GeometricMedian('{}'));"


def create_measurement_table(conn, cur):
    """
    Create table for individual observations
    """

    cur.execute(
        'CREATE TABLE IF NOT EXISTS measurements('
        'object_key serial PRIMARY KEY,'
        'stamp timestamp NOT NULL,'  # timestamp of observation
        'convex_hull geometry,'  # estimated convex hull
        'tracking_id varchar,'   # tracking ID of bounding box 
        'label varchar,'   # TODO: use it for topK DL predictions
        'centroid_3d geometry);' # convex hull centroid
    )
    conn.commit()


