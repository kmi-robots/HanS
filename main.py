from params import get_parser
import sys
import rclpy
from rosnodes.ObjectRecognition import ObjectRecognition

def main():

    # Load parameters and maintain as dictionary
    parser = get_parser()
    args_dict, unknown = parser.parse_known_args()

    if args_dict.mode=='train':
        #TODO train TPU model on target object classes
        pass
    elif args_dict.mode=='test':
        rclpy.init(args=sys.argv)
        rgbd_serv = ObjectRecognition(args_dict)
        rclpy.spin(rgbd_serv)

        rgbd_serv.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
    sys.exit(0)