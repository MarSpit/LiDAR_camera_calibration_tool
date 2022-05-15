from user_input_calibration import usr_input_calibration
from image_pcd_overlay_subscriber_publisher import Img_PCD_Subscriber_Overlay_Publisher
import rclpy
import params
import slider
import threading

def main(args=None):
    # Starting the slider in a different thread
    threading.Thread(target = slider.create_slider).start()
    # Running the subscriber publisher node
    rclpy.init(args=args)
    img_pcd_sub_overlay_pub = Img_PCD_Subscriber_Overlay_Publisher()
    rclpy.spin(img_pcd_sub_overlay_pub)
    rclpy.shutdown()

if __name__ == '__main__':
    main()