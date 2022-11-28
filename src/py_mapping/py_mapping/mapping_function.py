import numpy as np
import cv2

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseStamped, Pose, Point
from nav_msgs.msg import MapMetaData
from std_msgs.msg import Header

from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer

import message_filters

# CONSTANTS
RESOLUTION_MAP = 0.01
NAME_MAP = 'house'


class Minimal_PubSub(Node):
    def __init__(self):
        super().__init__('minimal_pubsub')

        qos_policy_pub = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.RELIABLE, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        self.Publisher_img = self.create_publisher(msg_type=OccupancyGrid, topic='grid_map', qos_profile=qos_policy_pub)
        self.Publisher_pose = self.create_publisher(msg_type=PoseStamped, topic='pose_robot', qos_profile=qos_policy_pub)

        # Subscribe to messages with message_filters due to synchronization of both topics.
        # Otherwise i had problems that actual pose and lidar scan data aren't from the same moment/position
        qos_policy_sub = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)
        Subscription_odom= message_filters.Subscriber(self, Odometry, '/odom', qos_profile=qos_policy_sub)
        Subscription_scan = message_filters.Subscriber(self, LaserScan, '/scan', qos_profile=qos_policy_sub)
        ts = message_filters.ApproximateTimeSynchronizer([Subscription_scan, Subscription_odom], 10, 0.01)
        ts.registerCallback(self.callback)

        # Variables for reading the transformation from the /tf topic
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.map = self.create_map()
    
    def callback(self, msg_scan, msg_odom):
        """ Callback function for Subscriber """
        try:
            # Read sensor values
            self.read_sensor_scan(msg_scan)
            self.read_sensor_odom(msg_odom)

            # Mapping
            self.fct_mapping()

            # Write Sensor Values
            self.write_sensor()
        except:
            print("Error - Waiting that everything is started..")
    
    def read_sensor_scan(self, msg_scan):
        """ Read sensor message from /scan topic """
        ranges = msg_scan.ranges

        lst_tmp = []
        for i in range(len(ranges)):
            if ranges[i] != float('inf'):
                # Angle for the i-th distance
                ang = i * msg_scan.angle_increment
                # Calculate distance
                if (ranges[i] > msg_scan.range_max):
                    r = msg_scan.range_max
                elif (ranges[i] < msg_scan.range_min):
                    r = msg_scan.range_min
                else:
                    r = ranges[i]
              
                lst_tmp.append([r, ang])
        self.np_LidarData = np.array(lst_tmp)

    def read_sensor_odom(self, msg_odom):
        """ Read sensor message from /odom topic """
        self.t = msg_odom.pose.pose.position
        self.q = msg_odom.pose.pose.orientation
        _, _, theta = self.euler_from_quaternion(self.q)

        self.np_Odometry = np.array([self.t.x, self.t.y, theta])


    def euler_from_quaternion(self, quaternion):
        """
        Converts quaternion to euler - roll, pitch, yaw

        Source: https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
        """
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = np.arctan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = np.arcsin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = np.arctan2(t3, t4)

        return roll_x, pitch_y, yaw_z

    def create_map(self):
        """ 
        Create the map with limits in meters and a resolution 
        """
        if NAME_MAP == 'world':
            self.x_lim = [-4, 4]
            self.y_lim = [-4, 4]
        else:
            self.x_lim = [-10, 10]
            self.y_lim = [-10, 10]

        x = np.arange(start = self.x_lim[0], stop = self.x_lim[1] + RESOLUTION_MAP, step = RESOLUTION_MAP)
        y = np.arange(start = self.y_lim[0], stop = self.y_lim[1] + RESOLUTION_MAP, step = RESOLUTION_MAP)
        return 0 * np.ones(shape=(len(x), len(y)), dtype=np.int8)
        
    def discretize(self, x_cont, y_cont):
        """
		Discretize continious x and y 
		"""
        x = int((x_cont - self.x_lim[0]) / RESOLUTION_MAP)
        y = int((y_cont - self.y_lim[0]) / RESOLUTION_MAP)

        return (x, y)

    def fct_mapping(self):
        """ Map the lidar data on the map """
        # Set lidar points in map
        for r, theta in self.np_LidarData:
            x = self.np_Odometry[0] + r * np.cos(theta + self.np_Odometry[2])
            y = self.np_Odometry[1] + r * np.sin(theta + self.np_Odometry[2])

            x, y = self.discretize(x, y)
            self.map[x, y] = 100


    def write_sensor(self):
        """ Write sensor data to topic /grid_map and actual robot position to /pose_robot topic """

        t = self.tf_buffer.lookup_transform('odom', 'odom', rclpy.time.Time())
        temp_x = ((self.map.shape[0]/2) * RESOLUTION_MAP)   # Calculate translation for map x-coordinate
        temp_y = ((self.map.shape[1]/2) * RESOLUTION_MAP)   # Calculate translation for map y-coordinate

        # Build OccupancyGrid-message
        pose = Pose(position=Point(x=-temp_x, y=-temp_y, z=t.transform.translation.z), orientation=t.transform.rotation)
        header = Header(stamp=Node.get_clock(self).now().to_msg(), frame_id="map")
        info = MapMetaData(width=self.map.shape[0], height=self.map.shape[1], resolution=RESOLUTION_MAP, map_load_time=Node.get_clock(self).now().to_msg(), origin=pose)
        msg_gridmap = OccupancyGrid(header=header, info=info)
        tmp = cv2.flip(self.map, 1)
        tmp = cv2.rotate(tmp, cv2.ROTATE_90_COUNTERCLOCKWISE)
        msg_gridmap.data = tmp.flatten().tolist()

        # Build PoseStamped-message 
        pose = Pose(position=Point(x=self.t.x, y=self.t.y, z=0.), orientation=self.q)
        header = Header(stamp=Node.get_clock(self).now().to_msg(), frame_id="map")
        msg_pose = PoseStamped(header=header, pose=pose)
        
        # Publish the map and pose to their topics
        self.Publisher_img.publish(msg_gridmap)
        self.Publisher_pose.publish(msg_pose)

        # Visualize the map with opencv -- for Debugging
        # cv2.imshow("Map", cv2.rotate(self.map, cv2.ROTATE_180))
        # cv2.waitKey(1)



def main(args=None):

    rclpy.init(args=args)
    minimal_PubSub = Minimal_PubSub()

    rclpy.spin(minimal_PubSub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_PubSub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()