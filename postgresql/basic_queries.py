
insert_measurement = "INSERT INTO measurements(stamp, label, robot_position," \
                         " convex_hull, centroid_3d) VALUES (" \
                         "'{}', '{}', ST_MakePoint({}, {}, {}), ST_GeomFromEWKT('{}'), ST_Centroid('{}'));"








