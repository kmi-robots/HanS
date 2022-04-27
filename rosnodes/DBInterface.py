from rclpy.node import Node
from datetime import datetime

from boundingbox_msg.msg import ConvexHull
from postgresql.io import *
from postgresql.basic_queries import *


class DBInterface(Node):

    def __init__(self, params):

        super().__init__('DBInterface')
        # Init postgresql table for single record/ convex hulls measured
        self.connection, self.cursor = connect_db(params.dbuser, params.dbname)
        # create_measurement_table(self.connection,self.cursor)

        # Listen to convex hull msgs
        self.subscription = self.create_subscription(ConvexHull, params.chull_topic, self.callback, qos_profile=10)

    def callback(self, msg: ConvexHull):
        # Convert convex hull msg to polyhedron and insert into table measurements
        if msg.polygons:
            ts = datetime.fromtimestamp(msg.header.stamp.sec + msg.header.stamp.nanosec / 1000000000)
            rx = msg.robot_pose.pose.position.x
            ry = msg.robot_pose.pose.position.y
            rz = msg.robot_pose.pose.position.z

            multipoint = 'MULTIPOINT ('
            # ((0 0 0, 0 1 0, 1 1 0, 1 0 0, 0 0 0)),
            for poly in msg.polygons:
                for p in poly.points:
                    multipoint += '{} {} {}, '.format(p.x, p.y, p.z)
            multipoint = multipoint[:-2] + ')'
            query = insert_measurement.format(ts.isoformat(), msg.label, rx, ry, rz, multipoint, multipoint)

            self.cursor.execute(query)
            self.connection.commit()
            print("convex hull added to db")
