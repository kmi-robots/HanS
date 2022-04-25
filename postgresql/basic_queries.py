
insert_measurement = "INSERT INTO measurements(stamp, tracking_id, label," \
                         " convex_hull, centroid_3d) VALUES (" \
                         "'{}', '{}', '{}', ST_GeomFromEWKT('{}'), ST_GeometricMedian('{}'));"








