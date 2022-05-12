""" Code for manual annotation of forbidden areas given a map
Takes name of area type as parameter, e.g., fire_escape_area
Same as draw_walls.py but for multi-point areas
Add points to polygon at each click
Use right click to close polygon at lastly clicked point

"""


import cv2
import math
import sys

from params import get_parser
from postgresql.draw_walls import load_map_yaml
from postgresql.io import *


class AreaGUI():

    def __init__(self, args, atype):
        self.user = args.dbuser
        self.dbname = args.dbname
        self.area_type = atype
        self.map_img = cv2.imread(args.map_img)
        self.map_h, self.map_w = self.map_img.shape[:2]
        self.map_config = load_map_yaml(args.map_cfg)
        self.origin = self.map_config['origin']
        self.x, self.y = self.origin[:2]
        self.resolution = self.map_config['resolution']
        self.translated_points = []
        self.points = []

    def run(self):
        self.conn, self.cur = connect_db(self.user, self.dbname)  # connect to db

        height, width = self.map_img.shape[:2]

        x0 = math.floor(math.fabs(self.x) / self.resolution)
        y0 = height - math.floor(math.fabs(self.y) / self.resolution)

        cv2.circle(self.map_img, (x0, y0), 3, (0, 0, 255), 3, cv2.FILLED)
        cv2.imshow('Image', self.map_img)

        cv2.setMouseCallback('Image', self.click_event)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

        disconnect_DB(self.conn, self.cur)

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(self.map_img, (x, y), 3, (0, 0, 255), 3, cv2.FILLED)
            self.points.append((x, y))
            xm = self.x + x * self.resolution
            ym = self.y + (self.map_h - y) * self.resolution
            self.translated_points.append((xm, ym))
            print("coordinates: (%f, %f)" % (xm, ym))
            if len(self.points) >= 2:
                cv2.line(self.map_img, self.points[-1], self.points[-2], (255, 0, 0), 5, cv2.LINE_AA)
        if event == cv2.EVENT_RBUTTONDOWN:
            cv2.line(self.map_img, self.points[-1], self.points[0], (255, 0, 0), 5, cv2.LINE_AA)
            self.translated_points.append(self.translated_points[0])
            self.insert_areas(self.translated_points, self.conn, self.cur)
            self.points.clear()
            self.translated_points.clear()
        cv2.imshow('Image', self.map_img)


    def insert_areas(self, points_list, connection, cursor):
        query = "INSERT INTO sw_areas(area, stamp, area_type) VALUES(ST_MakePolygon('LINESTRING("

        for p in points_list:
            query = query + str(p[0]) + " " + str(p[1]) + ","
        query = query[:-1]
        query = query + ")'), CURRENT_TIMESTAMP, '"+self.area_type+"');"

        cursor.execute(query)
        connection.commit()


def main():
    # Load parameters and maintain as dictionary
    parser = get_parser()
    args_dict, unknown = parser.parse_known_args()
    area_type = "waste_area" #"fire_escape_area" #"fire_call_point" #"waste_area" #type_of_area_annotated

    wg = AreaGUI(args_dict, area_type)
    wg.run()


if __name__ == "__main__":
    main()
    sys.exit(0)