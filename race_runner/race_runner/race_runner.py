import numpy as np
import os

from simple_pid import PID

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Point
from sensor_msgs.msg import LaserScan
from sensor_msgs.msg import PointCloud2
from visualization_msgs.msg import Marker
from laser_geometry import LaserProjection
from sensor_msgs_py import point_cloud2

class WallFollower(Node):

    def __init__(self):
        super().__init__('wall_follower')
        self.max_distance = 1.0
        self.view_angle = 60
        self.front_angle = 30
        self.points_in_wall = 2
        self.wall_to_follow = 0  # left:0 right 2
        self.distance_to_wall = 0.25
        self.pid_dist = PID(-1, -0.1, -0.05, setpoint=self.distance_to_wall, output_limits=(-0.1, 0.1))
        self.pid_angle = PID(-0.5, -0.0, -0.0, setpoint=np.pi/2, output_limits=(-0.5, 0.5))
        self.sides = ['left', 'rear', 'right', 'front']
        self.colors = [[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0], [1.0, 1.0, 0.0]]
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.pub_cmd_vel = self.create_publisher(Twist, "cmd_vel", 10)
        self.pub_debug = self.create_publisher(Twist, "debug", 10)
        self.pub_pc = self.create_publisher(PointCloud2, "point_cloud", 10)
        self.pub_viz = self.create_publisher(Marker, "visualization_marker", 10)

    def listener_callback(self, msg):
        if not os.path.isfile("/tmp/RUN"):
            return
        can_see_walls = np.zeros((4), dtype=bool)
        closest_object = np.zeros((4), dtype=np.float32)
        walls_angle = np.zeros((4), dtype=np.float32)
        range = msg.ranges
        range = np.array(range)
        range_size = len(range)
        rangefiltered = np.empty([2, range_size])
        temp = np.arange(start=0, stop=2*np.pi+msg.angle_increment, step=msg.angle_increment)
        rangefiltered[0] = temp[0:len(rangefiltered[0])]
        rangefiltered[1] = range
        # keep only data points that are within max_distance
        rangefiltered = rangefiltered[:,rangefiltered[1] < self.max_distance]
        rangefiltered = rangefiltered[:,~np.isnan(rangefiltered[1])]

        sides_views = []
        sides_views.append([np.radians(360 - self.view_angle/2), np.radians(self.view_angle/2)])
        sides_views.append([np.radians(90 - self.view_angle/2), np.radians(90 + self.view_angle/2)])
        sides_views.append([np.radians(180 - self.view_angle/2), np.radians(180 + self.view_angle/2)])
        # restrict front view
        sides_views.append([np.radians(270 - self.front_angle/2), np.radians(270 + self.view_angle/2)])

        for i in np.arange(len(sides_views)):
            if i>0:
                points = rangefiltered[:,(rangefiltered[0]>sides_views[i][0]) & (rangefiltered[0]<sides_views[i][1])]
            else:
                points = rangefiltered[:,(rangefiltered[0]>sides_views[i][0]) | (rangefiltered[0]<sides_views[i][1])]
            # get distance to closest point
            if points.shape[1] > 0:
                closest_object[i] = points[1][points[1].argmin()]
            line = self.find_lines(i, points)
            if line.shape[1] >= self.points_in_wall:
                can_see_walls[i] = True
                walls_angle[i] = np.arctan2(line[1][0] - line[1][-1], line[0][0] - line[0][-1])
            #print("%s: %s" % (self.sides[i], line.shape))

        #print("%s %s %s %s" % (self.sides, can_see_walls, closest_object, walls_angle))
        #import pdb
        #pdb.set_trace()
        self.find_direction(can_see_walls, closest_object, walls_angle)

    def find_direction(self, can_see_walls, closest_object, walls_angle):
        if can_see_walls[self.wall_to_follow]:
            state = 'follow'
        else:
            angle_diff = None
            distance_diff = None
            state = 'goal-seek'
        #print("%s %s %s" % (state, angle_diff, distance_diff))
        if state == 'goal-seek':
            linear_x = 0.1
            linear_y = 0.0
            angular_z = 0.0
        if state == 'follow':
            linear_x = 0.1
            linear_y = self.pid_dist(closest_object[self.wall_to_follow])
            angular_z = self.pid_angle(walls_angle[self.wall_to_follow])
        #print("%s %s %s" % (state, closest_object[self.wall_to_follow], linear_y))
        print("%s %s %s" % (state, walls_angle[self.wall_to_follow], angular_z))
        msg = Twist()
        msg.linear.x = 0.2
        msg.linear.y = linear_y
        #msg.linear.y = 0.0
        msg.angular.z = angular_z
        #msg.angular.z = 0.0
        #if closest_object[3] < self.distance_to_wall:
        #    print('obstacle')
        #    msg.linear.x = 0.0
        self.pub_cmd_vel.publish(msg)
        msg = Twist()
        msg.linear.x = float(walls_angle[self.wall_to_follow])
        msg.linear.y = 10 * angular_z
        self.pub_debug.publish(msg)

    def find_lines(self, side, points):
        if points.shape[1] == 0:
            return np.zeros((2,0), dtype=np.float32)

        # Cache some resuable values
        pts_x = points[1] * np.cos(points[0])
        pts_y = points[1] * np.sin(points[0])

        voting_points_x = []
        voting_points_y = []
        # take first and last point in filtered scan
        voting_points_x.append(pts_x[0])
        voting_points_y.append(pts_y[0])
        voting_points_x.append(pts_x[-1])
        voting_points_y.append(pts_y[-1])

        if(len(voting_points_x)) == 0:
            print("%s: no voting point" % self.sides[side])
            return np.zeros((2,0), dtype=np.float32)

        voting_points_x = np.array(voting_points_x)
        voting_points_y = np.array(voting_points_y)

        # laser scan directon is set to counterclockwise in launch file ld06.launch.py
        # this corresponds to angle orientation in x/y plane
        marker = Marker()
        marker.header.frame_id = "base_laser"
        marker.ns = self.sides[side]
        marker.id = 0
        marker.action = Marker.ADD
        marker.type = Marker.LINE_LIST
        x_min = voting_points_x.argmin()
        x_max = voting_points_x.argmax()
        p = Point()
        p.x = voting_points_x[x_min]
        p.y = voting_points_y[x_min]
        marker.points.append(p)
        p = Point()
        p.x = voting_points_x[x_max]
        p.y = voting_points_y[x_max]
        marker.points.append(p)

        marker.scale.x = 0.01
        marker.scale.y = 0.01
        marker.scale.z = 0.0

        marker.color.r = self.colors[side][0]
        marker.color.g = self.colors[side][1]
        marker.color.b = self.colors[side][2]
        marker.color.a = 1.0

        self.pub_viz.publish(marker)

        return np.array([voting_points_x, voting_points_y])


def main(args=None):
    rclpy.init(args=args)

    wf = WallFollower()

    rclpy.spin(wf)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    wf.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
