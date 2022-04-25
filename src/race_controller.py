#!/usr/bin/env python
import rospy
import numpy as np

from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import PoseArray
from utilities.controllers import PurePursuit
from utilities.Trajectory import LinearTrajectory

class RaceController():
    """
    A controller for parking in front of a cone.
    Listens for a relative cone location and publishes control commands.
    Can be used in the simulator and on the real robot.
    """
    def __init__(self):

        DRIVE_TOPIC = rospy.get_param("~drive_topic") # set in launch file; different for simulator vs racecar
        self.drive_pub = rospy.Publisher(DRIVE_TOPIC,
            AckermannDriveStamped, queue_size=10)

        self.traj_sub = rospy.Subscriber("/trajectory", PoseArray, self.trajectory_callback, queue_size=1)

        self.pursuit = PurePursuit(0.325)
        self.MAX_SPEED = 7
    
    def trajectory_callback(self, msg):
        ''' Clears the currently followed trajectory, and loads the new one from the message
        '''
        print ("Receiving new trajectory:", len(msg.poses), "points")
        if len(msg.poses) == 0:
            self.drive(0,0, None, None)
            return
        
        knots = []
        for p in msg.poses:
            knots.append([p.position.x, p.position.y])
        knots = np.array(knots)

        t_breaks = np.arange(knots.shape[0])

        traj = LinearTrajectory(t_breaks, knots)

        #steer, speed = self.pursuit.adaptiveControl(traj, self.v_function)
        steer = self.pursuit.control(traj, Lfw = 2)
        speed = 0 #self.MAX_SPEED
        print("steer: ", steer)

        # TODO the speed computed is in the relative reference frame, thus it has the wrong signs
        drive_cmd = self.drive(steer, abs(speed), None, None)


        self.drive_pub.publish(drive_cmd)


    def v_function(self, v_desired, traj):
        #adaptive velocity function
        Lfw, lfw = 0, 0
        v_desired = abs(v_desired)
        if v_desired < 2:
            Lfw = v_desired
        elif v_desired >=2 and v_desired < 6:
            Lfw = 1.5*v_desired
        else:
            Lfw = 12
        return Lfw, lfw
    
    def drive(self, theta, speed, theta_dot = None,  acceleration = None):
        """
            Takes in steering and speed commands for the car.
            :param theta: steering angle [rad]. right sided
            :type theta: float
            :param theta_dot: steering angular velocity [rad/s]
            :type theta_dot: float
            :param speed: speed in [m/s]
            :type speed: float

            :param speed: speed in [m/s]
            :type speed: float
        """
        ack_msg = AckermannDriveStamped()
        ack_msg.header.stamp = rospy.get_rostime()
        ack_msg.header.frame_id = 'base_link'
        ack_msg.drive.steering_angle = np.clip(theta, -0.34, 0.34)
        ack_msg.drive.speed = speed
        if not theta_dot is None:
            ack_msg.drive.steering_angle_velocity = theta_dot
        else:
            ack_msg.drive.steering_angle_velocity = 0.1
        if not acceleration is None:
            ack_msg.drive.acceleration = acceleration
        return ack_msg

if __name__ == '__main__':
    try:
        rospy.init_node('RaceController', anonymous=True)
        RaceController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
