#!/usr/bin/python
import rospy
import roslib
import gopigo   
import math 
import numpy

# Messages
from std_msgs.msg import Float32

# Query GoPiGo robot for left and right wheel encoders.
# Publish the estimated left and right angular wheel velocities
class WheelEncoderPublisher:
  def __init__(self):
    rospy.init_node('gopigo_state_updater')
    # Read in tangential velocity targets
    self.lwheel_angular_vel_motor_sub = rospy.Subscriber('lwheel_angular_vel_motor', Float32, self.lwheel_angular_vel_motor_callback)
    self.rwheel_angular_vel_motor_sub = rospy.Subscriber('rwheel_angular_vel_motor', Float32, self.rwheel_angular_vel_motor_callback)

    self.lwheel_angular_vel_enc_pub = rospy.Publisher('lwheel_angular_vel_enc', Float32, queue_size=10)
    self.rwheel_angular_vel_enc_pub = rospy.Publisher('rwheel_angular_vel_enc', Float32, queue_size=10)
    self.rate = rospy.get_param('~rate', 10)
    self.err_tick_incr = rospy.get_param('~err_tick_incr',20) # Filter out clearly erroneous encoder readings
    self.lwheel_encs = [gopigo.enc_read(1)]*5
    self.rwheel_encs = [gopigo.enc_read(0)]*5
    self.time_prev_update = rospy.Time.now();

    self.R = rospy.get_param('~robot_wheel_radius', .03)

    # Need a little hack to incorporate direction wheels are spinning
    self.lwheel_dir = 1;
    self.rwheel_dir = 1;

#    self.lwheel_ticks_history = [self.lwheel_ticks]*3
#    self.rwheel_ticks_history = [self.rwheel_ticks]*3

  # Really bad hack to get motor spin direction
  def lwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.lwheel_dir = 1
    else: self.lwheel_dir = -1
  # Really bad hack to get motor spin direction
  def rwheel_angular_vel_motor_callback(self,msg):
    if msg.data >= 0: self.rwheel_dir = 1
    else: self.rwheel_dir = -1

  def enc_2_rads(self,enc_cms):
    prop_revolution = (enc_cms) / (2.0*math.pi*self.R)
    rads =  prop_revolution * (2*math.pi)
    return rads

  def update(self):
    print self.lwheel_dir, self.rwheel_dir
    lwheel_enc = self.lwheel_dir * gopigo.enc_read(1) * .01 # cm's moved
    rwheel_enc = self.rwheel_dir * gopigo.enc_read(0) * .01 # cm's moved
#    print lwheel_enc, rwheel_enc
#    lwheel_ticks = gopigo.enc_read(1) # Actually cm moved
#    rwheel_ticks = gopigo.enc_read(0)

#    print lwheel_ticks, rwheel_ticks

    self.lwheel_encs = self.lwheel_encs[1:] + [lwheel_enc]
    self.rwheel_encs = self.rwheel_encs[1:] + [rwheel_enc]

    # History of past three encoder reading
#    self.lwheel_ticks_history = self.lwheel_ticks_history[1:] + [lwheel_ticks]
#    self.rwheel_ticks_history = self.rwheel_ticks_history[1:] + [rwheel_ticks]

#    lwheel_ticks_delta = lwheel_ticks - self.lwheel_ticks
#    rwheel_ticks_delta = rwheel_ticks - self.rwheel_ticks

    # Skip clearly errorneous readings
#    lwheel_ticks_avg_delta = numpy.mean(map(lambda x: abs(x - numpy.mean(self.lwheel_ticks_history)), self.lwheel_ticks_history))
#    rwheel_ticks_avg_delta = numpy.mean(map(lambda x: abs(x - numpy.mean(self.rwheel_ticks_history)), self.rwheel_ticks_history))

#    print self.lwheel_ticks_history, self.rwheel_ticks_history, lwheel_ticks_avg_delta, rwheel_ticks_avg_delta

#    if lwheel_ticks_avg_delta > self.err_tick_incr or rwheel_ticks_avg_delta > self.err_tick_incr: return

    #print lwheel_ticks, rwheel_ticks

    time_curr_update = rospy.Time.now()
    dt = (time_curr_update - self.time_prev_update).to_sec()

    # Compute angular velocity in rad/s
    lwheel_enc_delta = self.lwheel_encs[-1] - self.lwheel_encs[-2]
    rwheel_enc_delta = self.rwheel_encs[-1] - self.rwheel_encs[-2]
    lwheel_angular_vel_enc = self.enc_2_rads(lwheel_enc_delta) / dt
    rwheel_angular_vel_enc = self.enc_2_rads(rwheel_enc_delta) / dt
#    lwheel_angular_vel_enc = ( lwheel_ticks_delta / dt ) * ((2.0 * math.pi) / self.N)
#    rwheel_angular_vel_enc = ( rwheel_ticks_delta / dt ) * ((2.0 * math.pi) / self.N)
    self.lwheel_angular_vel_enc_pub.publish(lwheel_angular_vel_enc)
    self.rwheel_angular_vel_enc_pub.publish(rwheel_angular_vel_enc)

#    self.lwheel_ticks = lwheel_ticks
#    self.rwheel_ticks = rwheel_ticks

    self.time_prev_update = time_curr_update

  def spin(self):
    rospy.loginfo("Start gopigo_state_updater")
    rate = rospy.Rate(self.rate)
    rospy.on_shutdown(self.shutdown)

    while not rospy.is_shutdown():
      self.update();
      rate.sleep()
    rospy.spin()

  def shutdown(self):
    rospy.loginfo("Stop gopigo_state_updater")
    # Stop message
    self.lwheel_angular_vel_enc_pub.publish(0)
    self.rwheel_angular_vel_enc_pub.publish(0)
    rospy.sleep(1)

def main():
  encoder_publisher = WheelEncoderPublisher();
  encoder_publisher.spin()

if __name__ == '__main__':
  main(); 


