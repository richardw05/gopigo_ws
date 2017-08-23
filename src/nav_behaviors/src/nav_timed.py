#!/usr/bin/env python

# Direct robot to move forward via geometry_msgs/Twist messages for some time duration

import rospy
from geometry_msgs.msg import Twist

class NavTimed():
  def __init__(self):
    rospy.init_node('nav_timed')
    self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    self.time_start = rospy.Time.now()
    self.time_to_run = rospy.get_param('~time_to_run',10)
    self.v_target = rospy.get_param('~v_target',0.1)
    self.w_target = rospy.get_param('~w_target',0)    

  def update(self):
    time_duration = (rospy.Time.now() - self.time_start).to_sec()
    if time_duration > self.time_to_run:
      rospy.signal_shutdown('Done nav_forward')

    move_cmd = Twist()
    move_cmd.linear.x = self.v_target
    move_cmd.angular.z = self.w_target
    self.cmd_vel_pub.publish(move_cmd)

  def spin(self):
    rospy.loginfo("Start nav_forward")
    rate = rospy.Rate(10)
    rospy.on_shutdown(self.shutdown)
    while not rospy.is_shutdown():
      self.update()
      rate.sleep()
    rospy.spin()

  def shutdown(self):
    rospy.loginfo("Stop nav_forward")
    # Stop message
    self.cmd_vel_pub.publish(Twist())
    rospy.sleep(1)

def main():
  nav_timed = NavTimed();
  nav_timed.spin()

if __name__ == '__main__':
  main()

