from params import get_parser
import sys
import rclpy

from rosnodes.DBInterface import DBInterface

def main():

    # Load parameters and maintain as dictionary
    parser = get_parser()
    args_dict, unknown = parser.parse_known_args()

    rclpy.init(args=sys.argv)

    db_interface_node = DBInterface(args_dict)
    rclpy.spin(db_interface_node)

    db_interface_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    sys.exit(0)