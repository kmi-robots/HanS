""" Code for manual wall annotation given a map
"""

import yaml
import sys
import cv2
import math

from postgresql.io import *
from params import get_parser

def load_map_yaml(path_to_yaml):

    with open(path_to_yaml, 'r') as yaml_in:
        config_dict = yaml.safe_load(yaml_in)
    return config_dict

class WallGUI():

    def __init__(self, args):
        self.user = args.dbuser
        self.dbname = args.dbname
        self.map_img = cv2.imread(args.map_img)
        self.map_h, self.map_w = self.map_img.shape[:2]
        self.map_config = load_map_yaml(args.map_cfg)
        self.origin = self.map_config['origin']
        self.x, self.y = self.origin[:2]
        self.resolution = self.map_config['resolution']
        self.points = []
        self.wheight = args.wall_height

    def run(self):

        self.conn, self.cur = connect_db(self.user, self.dbname) # connect to db

        x0 = math.floor(math.fabs(self.x) / self.resolution)
        y0 = self.map_w - math.floor(math.fabs(self.y) / self.resolution)
        cv2.circle(self.map_img, (x0, y0), 3, (0, 0, 255), 3, cv2.FILLED)
        cv2.imshow('Image', self.map_img)

        cv2.setMouseCallback('Image', self.click_event)
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        disconnect_DB(self.conn,self.cur)

    def click_event(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            cv2.circle(self.map_img, (x, y), 3, (0, 0, 255), 3, cv2.FILLED)
            self.points.append((x, y))
            if len(self.points) >= 2:
                cv2.line(self.map_img, self.points[-1], self.points[-2], (255, 0, 0), 5, cv2.LINE_AA)
                xm, ym = self.points[-1]
                xm = self.x + xm * self.resolution
                ym = self.y + (self.map_h - ym) * self.resolution
                xm_, ym_ = self.points[-2]
                xm_ = self.x + xm_ * self.resolution
                ym_ = self.y + (self.map_h - ym_) * self.resolution
                print("coordinates: p1 (%f, %f), p2 (%f, %f)" % (xm, ym, xm_, ym_))
                self.insert_wall(xm, ym, xm_, ym_, wheight=self.wheight)
                self.points.clear()
            cv2.imshow('Image', self.map_img)

    def insert_wall(self, xm, ym, xm_, ym_, wheight=4):
        query_mask = "INSERT INTO sw_walls(surface) " \
                     "VALUES(ST_Extrude(ST_GeomFromText('" \
                     "LINESTRING({} {}, {} {})'),0,0,{}))"
        query = query_mask.format(xm, ym, xm_, ym_, wheight)
        self.cur.execute(query)
        self.conn.commit()

def main():
    # Load parameters and maintain as dictionary
    parser = get_parser()
    args_dict, unknown = parser.parse_known_args()

    wg = WallGUI(args_dict)
    wg.run()


if __name__ == "__main__":
    main()
    sys.exit(0)
