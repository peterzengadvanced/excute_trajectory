#!/usr/bin/env python3
import rospy
import os
import numpy as np
from gazebo_msgs.msg import ModelState
from geometry_msgs.msg import Pose, Twist
import tf.transformations as tft

def load_trajectory(file_path):
    return np.loadtxt(file_path)

def publish_model_state(trajectory, publisher, model_name='stage1', reference_frame='world', rate_hz=10):
    rate = rospy.Rate(rate_hz)
    for waypoint in trajectory:
        x, y, z, roll, pitch, yaw = waypoint
        quat = tft.quaternion_from_euler(roll, pitch, yaw)
        state_msg = ModelState()
        state_msg.model_name = model_name
        state_msg.pose.position.x = x
        state_msg.pose.position.y = y
        state_msg.pose.position.z = z
        state_msg.pose.orientation.x = quat[0]
        state_msg.pose.orientation.y = quat[1]
        state_msg.pose.orientation.z = quat[2]
        state_msg.pose.orientation.w = quat[3]
        # Setting twist to zero to simply reposition the model
        state_msg.twist = Twist()
        state_msg.reference_frame = reference_frame
        publisher.publish(state_msg)
        rospy.loginfo("Published state: x=%.3f, y=%.3f, z=%.3f", x, y, z)
        rate.sleep()

def main():
    rospy.init_node('model_state_executor', anonymous=True)
    trajectory_file = "/home/peterzeng/hapticanddmp/besttrajectory/stage1trajectory/forpose05/generate05.txt"
    
    pub = rospy.Publisher('/gazebo/set_model_state', ModelState, queue_size=10)
    rospy.sleep(1.0)  # Allow time for publisher to connect

    while pub.get_num_connections() == 0 and not rospy.is_shutdown():
        rospy.loginfo("Waiting for Gazebo to subscribe to /gazebo/set_model_state...")
        rospy.sleep(0.5)
    
    if not os.path.exists(trajectory_file):
        rospy.logerr("Trajectory file does not exist: %s", trajectory_file)
        return
    trajectory = load_trajectory(trajectory_file)
    
    rospy.loginfo("Starting trajectory execution for model state update.")
    publish_model_state(trajectory, pub, model_name='stage1', reference_frame='world', rate_hz=10)
    rospy.loginfo("Trajectory execution completed.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
