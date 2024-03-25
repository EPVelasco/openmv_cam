#!/usr/bin/env python2.7

# Python libraries
import sys, serial, struct, time

# Import OpenCV
import cv2
import numpy as np

# Import ROS
import rospy
import roslib
from sensor_msgs.msg import Image
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import Header

class openmv_cam_client(object):
    def __init__(self):
        self.port_name = rospy.get_param('~port_name', '/dev/ttyACM0')
        self.serial_port = None
        self.img = None
        self.img_aux = None
        self.key = 0
        self.bridge = CvBridge()
        self.pub_raw = rospy.Publisher("/openmv_cam/image/raw", Image, queue_size = 10)
        #self.pub_comp = rospy.Publisher("/image_compressed", CompressedImage, queue_size = 10)


    def initialize_serial(self):

        self.serial_port = serial.Serial(self.port_name, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
             xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)

        #self.serial_port = serial.Serial(self.port_name, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
        #         xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)


    def read_image(self):

        # Read data from the serial buffer
        self.serial_port.write("snap")
        self.serial_port.flush()
        size = struct.unpack('<L', self.serial_port.read(4))[0]
        buf = self.serial_port.read(size)

        # Use numpy to construct an array from the bytes
        x = np.fromstring(buf, dtype='uint8')

        # Decode the array into an image
        self.img_aux = cv2.imdecode(x, cv2.IMREAD_UNCHANGED)

        # Get the dimensions of the image
        (h, w) = self.img_aux.shape[:2]

        # Calculate the center of the image
        center = (w // 2, h // 2)

        # Define the rotation angle in degrees
        angle = -90  # Rotate by 90 degrees

        # Get the rotation matrix using OpenCV
        rotation_matrix = cv2.getRotationMatrix2D(center, angle, 1.0)

        # Perform the rotation
        self.img = cv2.warpAffine(self.img_aux, rotation_matrix, (w, h))

    def publish_image(self):

        """
        compressed = CompressedImage()
        compressed.header.stamp = rospy.Time.now()
        compressed.format = "jpeg"
        compressed.data = np.array(cv2.imencode('.jpg', self.img)[1]).tostring()
        self.pub_comp.publish(compressed)
        """
        #header = Header(stamp=rospy.Time.now())
        try:
            if np.size(self.img.shape) == 3:
                imgmsg = self.bridge.cv2_to_imgmsg(self.img, 'bgr8')
            else:
                imgmsg = self.bridge.cv2_to_imgmsg(self.img, 'mono8')

            # Crear un objeto Header y asignarle el timestamp actual
            
            imgmsg.header.stamp = rospy.Time.now()
            self.pub_raw.publish(imgmsg)
        except CvBridgeError as e:
            print(e)


    def show_image(self):
        cv2.imshow("Stream:", self.img)
        self.key = cv2.waitKey(20)

    def disconnect(self):
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            print("Disconnected successfully.")
        else:
            None

    def run(self):

        self.initialize_serial()

        #pub = initialize_publishers()

        rate = rospy.Rate(10) # Run at 10Hz

        while not rospy.is_shutdown():


            #start_time = time.time()
            self.read_image()
            #self.show_image()
            self.publish_image()            
            
            if self.key == 27:
                self.seial_port.close()
                cv2.destroyWindow("preview")          
                break

            rate.sleep()
            #print("FPS: ", 1.0/(time.time()-start_time))

def main(args, cam):
    rospy.init_node('OpenMV_cam', anonymous=True)
    cam.run()
    return None
# Main
if __name__ == '__main__':
    try:
        cam = openmv_cam_client()
        main(sys.argv, cam)
    except(rospy.ROSInterruptException, KeyboardInterrupt):
        print("Error System")
        cam.disconnect()
        pass
    else:
        cam.disconnect()
        print("Complete Execution")
        pass

    
         
