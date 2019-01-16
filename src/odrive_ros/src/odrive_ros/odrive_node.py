#!/usr/bin/env python
from __future__ import print_function

#import roslib; roslib.load_manifest('BINCADDY')
import rospy
import tf.transformations
import tf_conversions
import tf2_ros

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from dynamixel_msgs.msg import JointState
from dynamixel_msgs.msg import MotorState
import std_srvs.srv

import time
import math

from odrive_interface import ODriveInterfaceSerial, ODriveInterfaceAPI

class ROSLogger(object):
    """Imitate a standard Python logger, but pass the messages to rospy logging.
    """
    def debug(self, msg):    rospy.logdebug(msg)  #  print(msg) #
    def info(self, msg):     rospy.loginfo(msg)   #  print(msg) #
    def warn(self, msg):     rospy.logwarn(msg)   #  print(msg) #
    def error(self, msg):    rospy.logerr(msg)    #  print(msg) #
    def critical(self, msg): rospy.logfatal(msg)  #  print(msg) #
    
    # use_index = False (bool)
    # offset_float = 0.590887010098 (float)
    # calib_range = 0.019999999553 (float)
    # mode = 0 (int)
    # offset = 1809 (int)
    # cpr = 4096 (int)
    # idx_search_speed = 10.0 (float)
    # pre_calibrated = False (bool)

#m_s_to_rpm = 60.0/tyre_circumference
#m_s_to_erpm = 10 * m_s_to_rpm 

# 4096 counts / rev, so 4096 == 1 rev/s


# 1 m/s = 3.6 km/hr

class ODriveNode(object):
    last_speed = 0.0
    driver = None
    last_cmd_vel_time = None
    
    # Robot wheel_track params for velocity -> motor speed conversion
    wheel_track = None
    tyre_circumference = None
    encoder_counts_per_rev = None
    m_s_to_value = 0
    axis_for_right = 0
    
    # Startup parameters
    connect_on_startup = False
    calibrate_on_startup = False
    engage_on_startup = False
    
    def __init__(self):
        self.axis_for_right = float(rospy.get_param('~axis_for_right', 1)) # if right calibrates first, this should be 0, else 1
        self.wheel_track = float(rospy.get_param('~wheel_track', 0.285)) # m, distance between wheel centres
        self.tyre_circumference = float(rospy.get_param('~tyre_circumference', 0.341)) # used to translate velocity commands in m/s into motor rpm
        
        self.connect_on_startup   = rospy.get_param('~connect_on_startup', True)
        self.calibrate_on_startup = rospy.get_param('~calibrate_on_startup', True)
        self.engage_on_startup    = rospy.get_param('~engage_on_startup', True)
        
        self.max_speed   = rospy.get_param('~max_speed', 0.5)
        self.max_angular = rospy.get_param('~max_angular', 1.0) 
        
        self.publish_current = rospy.get_param('~publish_current', True)
        
        self.has_preroll     = rospy.get_param('~use_preroll', False)
                
        self.publish_current = rospy.get_param('~publish_current', True)
        
        self.publish_odom    = rospy.get_param('~publish_odom', True)
        self.publish_tf      = rospy.get_param('~publish_odom_tf', True)
        self.odom_topic      = rospy.get_param('~odom_topic', "odom")
        self.odom_frame      = rospy.get_param('~odom_frame', "odom")
        self.base_frame      = rospy.get_param('~base_frame', "base_link")
        self.odom_calc_hz    = rospy.get_param('~odom_calc_hz', 20)
	
	self.publish_joint_state    = rospy.get_param('~publish_joint_state', True)
	self.joint_state_topic      = rospy.get_param('~joint_state_topic', "/tilt_controller/state")
	self.Calibrate_Axis    = rospy.get_param('~Calibrate_Axis', 1)
	self.motor_id    = rospy.get_param('~motor_id', "0")
	self.sweep_factor    = rospy.get_param('~sweep_factor', 0.2)
        
        rospy.on_shutdown(self.terminate)

        rospy.Service('connect_driver',    std_srvs.srv.Trigger, self.connect_driver)
        rospy.Service('disconnect_driver', std_srvs.srv.Trigger, self.disconnect_driver)
    
        rospy.Service('calibrate_motors',  std_srvs.srv.Trigger, self.calibrate_motor)
        rospy.Service('engage_motors',     std_srvs.srv.Trigger, self.engage_motor)
        rospy.Service('release_motors',    std_srvs.srv.Trigger, self.release_motor)

        self.vel_subscribe = rospy.Subscriber("/tilt_controller/command", Float64, self.cmd_vel_callback, queue_size=2)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_check) # stop motors if no cmd_vel received > 1second
                
        if self.publish_current:
            self.current_loop_count = 0
            self.left_current_accumulator  = 0.0
            self.right_current_accumulator = 0.0
            self.current_timer = rospy.Timer(rospy.Duration(0.05), self.timer_current) # publish motor currents at 10Hz, read at 50Hz
            self.current_publisher_left  = rospy.Publisher('odrive/left_current', Float64, queue_size=2)
            self.current_publisher_right = rospy.Publisher('odrive/right_current', Float64, queue_size=2)
            rospy.loginfo("ODrive will publish motor currents.")
                    
        if self.publish_odom:
            rospy.Service('reset_odometry',    std_srvs.srv.Trigger, self.reset_odometry)
            
            self.odom_publisher  = rospy.Publisher(self.odom_topic, Odometry, queue_size=2)
            # setup message
            self.odom_msg = Odometry()
            #print(dir(self.odom_msg))
            self.odom_msg.header.frame_id = self.odom_frame
            self.odom_msg.child_frame_id = self.base_frame
            self.odom_msg.pose.pose.position.z = 0.0    # always on the ground, we hope
            self.odom_msg.pose.pose.orientation.x = 0.0 # always vertical
            self.odom_msg.pose.pose.orientation.y = 0.0 # always vertical
            self.odom_msg.twist.twist.linear.y = 0.0  # no sideways
            self.odom_msg.twist.twist.linear.z = 0.0  # or upwards... only forward
            self.odom_msg.twist.twist.angular.x = 0.0 # or roll
            self.odom_msg.twist.twist.angular.y = 0.0 # or pitch... only yaw
            
            # store current location to be updated. 
            self.x = 0.0
            self.y = 0.0
            self.theta = 0.0
            
            # setup transform
            self.tf_publisher = tf2_ros.TransformBroadcaster()
            self.tf_msg = TransformStamped()
            self.tf_msg.header.frame_id = self.odom_frame
            self.tf_msg.child_frame_id  = self.base_frame
            self.tf_msg.transform.translation.z = 0.0
            self.tf_msg.transform.rotation.x = 0.0
            self.tf_msg.transform.rotation.y = 0.0

            self.odom_timer = rospy.Timer(rospy.Duration(1/float(self.odom_calc_hz)), self.timer_odometry)
	
	if self.publish_joint_state:
            self.joint_state_publisher  = rospy.Publisher(self.joint_state_topic, JointState, queue_size=2)
            # setup message
            self.joint_state_msg = JointState()
	    self.joint_state_msg.name = self.motor_id

	    self.state_subscriber = rospy.Subscriber('/tilt_controller/motor_states/%s' % self.motor_id, MotorState, self.state_callback)
            # setup message
            self.state = MotorState()

	    self.RADIANS_PER_ENCODER_TICK = 2.0 * 3.1415926 / 4000.0
            self.ENCODER_TICKS_PER_RADIAN = 1.0 / self.RADIANS_PER_ENCODER_TICK
            #print(dir(self.odom_msg))
            # self.joint_state_msg.header.frame_id = self.odom_frame
            # self.joint_state_msg.child_frame_id = self.base_frame


        if not self.connect_on_startup:
            rospy.loginfo("ODrive node started, but not connected.")
            return
        
        if not self.connect_driver(None)[0]:
            return # Failed to connect
        
        if not self.calibrate_on_startup:
            rospy.loginfo("ODrive node started and connected. Not calibrated.")
            return
        
        if not self.calibrate_motor(None)[0]:
            return
            
        if not self.engage_on_startup:
            rospy.loginfo("ODrive connected and configured. Engage to drive.")
            return
        
        if not self.engage_motor(None)[0]:
            return
        
        rospy.loginfo("ODrive connected and configured. Ready to drive.")
        
    def terminate(self):
        if self.driver:
            self.driver.release()
        self.timer.shutdown()
    
    # ROS services
    def connect_driver(self, request):
        if self.driver:
            rospy.logerr("Already connected.")
            return (False, "Already connected.")
        
        self.driver = ODriveInterfaceAPI(calibrate_axis=self.Calibrate_Axis, logger=ROSLogger())
        rospy.loginfo("Connecting to ODrive...")
        if not self.driver.connect(right_axis=self.axis_for_right):
            self.driver = False
            rospy.logerr("Failed to connect.")
            return (False, "Failed to connect.")
            
        rospy.loginfo("ODrive connected.")
        
        self.m_s_to_value = self.driver.encoder_cpr/self.tyre_circumference
                
        if self.publish_odom:
	    if self.Calibrate_Axis != 1:
            	self.old_pos_l = self.driver.left_axis.encoder.pos_cpr
            else:
            	self.old_pos_r = self.driver.right_axis.encoder.pos_cpr
        
        return (True, "ODrive connected successfully")
    
    def disconnect_driver(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.disconnect():
            return (False, "Failed disconnection, but try reconnecting.")
        return (True, "Disconnection success.")
    
    def calibrate_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
            
        if self.has_preroll:
            if not self.driver.preroll():
                return (False, "Failed preroll.")        
        else:
            if not self.driver.calibrate():
                return (False, "Failed calibration.")
                
        return (True, "Calibration success.")
                    
    def engage_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.engage():
            return (False, "Failed to engage motor.")
        return (True, "Engage motor success.")
    
    def release_motor(self, request):
        if not self.driver:
            rospy.logerr("Not connected.")
            return (False, "Not connected.")
        if not self.driver.release():
            return (False, "Failed to release motor.")
        return (True, "Release motor success.")
    
    def reset_odometry(self, request):
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        return(True, "Odometry reset.")
    
    # Helpers and callbacks
    
    def convert(self, forward, ccw):
        angular_to_linear = ccw * (self.wheel_track/2.0) 
        left_linear_val  = int((forward - angular_to_linear) * self.m_s_to_value)
        right_linear_val = int((forward + angular_to_linear) * self.m_s_to_value)
    
        return left_linear_val, right_linear_val

    def cmd_vel_callback(self, msg):
        #rospy.loginfo("Received a /cmd_vel message!")
        #rospy.loginfo("Linear Components: [%f, %f, %f]"%(msg.linear.x, msg.linear.y, msg.linear.z))
        #rospy.loginfo("Angular Components: [%f, %f, %f]"%(msg.angular.x, msg.angular.y, msg.angular.z))

        # rostopic pub -r 1 /commands/motor/current std_msgs/Float64 -- -1.0

        # Do velocity processing here:
        # Use the kinematics of your robot to map linear and angular velocities into motor commands
        
        # 3600 ERPM = 360 RPM ~= 6 km/hr
        
        #angular_to_linear = msg.angular.z * (wheel_track/2.0) 
        #left_linear_rpm  = (msg.linear.x - angular_to_linear) * m_s_to_erpm
        #right_linear_rpm = (msg.linear.x + angular_to_linear) * m_s_to_erpm
        
        x = max(min(msg.data * self.sweep_factor, self.max_speed),   -self.max_speed)
        z = max(min(msg.data * self.sweep_factor, self.max_angular), -self.max_angular)
        
        left_linear_val, right_linear_val = self.convert(x,z)
        
        # if wheel speed = 0, stop publishing after sending 0 once. #TODO add error term, work out why VESC turns on for 0 rpm
        if self.last_speed == 0 and abs(left_linear_val) == 0 and abs(right_linear_val) == 0:
            return
        
        # Then set your wheel speeds (using wheel_left and wheel_right as examples)
        #self.left_motor_pub.publish(left_linear_rpm)
        #self.right_motor_pub.publish(right_linear_rpm)
        #wheel_left.set_speed(v_l)
        #wheel_right.set_speed(v_r)
        
        rospy.logdebug("Driving left: %d, right: %d, from linear.x %.2f and angular.z %.2f" % (left_linear_val, right_linear_val, msg.data * self.sweep_factor, msg.data * self.sweep_factor))
        self.driver.drive(left_linear_val, right_linear_val)

        self.last_speed = max(abs(left_linear_val), abs(right_linear_val))
	dt = 0.0
	if self.last_cmd_vel_time:
	    dt = rospy.Time.now().to_sec() - self.last_cmd_vel_time.to_sec();
        self.last_cmd_vel_time = rospy.Time.now()

	self.joint_state_msg.goal_pos = msg.data * dt;
	

    def state_callback(self, msg):
	if self.publish_joint_state:
	    self.joint_state_msg.motor_temps = 0
            self.joint_state_msg.goal_pos = self.raw_to_rad(state.goal, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
            self.joint_state_msg.current_pos = self.raw_to_rad(state.position, self.initial_position_raw, self.flipped, self.RADIANS_PER_ENCODER_TICK)
            self.joint_state_msg.error = state.error * self.RADIANS_PER_ENCODER_TICK
            self.joint_state_msg.velocity = state.speed * self.VELOCITY_PER_TICK
            self.joint_state_msg.load = state.load
            self.joint_state_msg.is_moving = state.moving
            self.joint_state_msg.header.stamp = rospy.Time.from_sec(state.timestamp)
                
            self.joint_state_publisher.publish(self.joint_state_msg)
                
    def timer_current(self, event):
        if not self.driver or not hasattr(self.driver, 'driver') or not hasattr(self.driver.driver, 'axis0'):
            return
        
	if self.Calibrate_Axis != 1:
            self.left_current_accumulator += self.driver.left_axis.motor.current_control.Ibus
	else:
            self.right_current_accumulator += self.driver.right_axis.motor.current_control.Ibus
        
        self.current_loop_count += 1
        if self.current_loop_count >= 9:
            # publish appropriate axis
	    if self.Calibrate_Axis != 1:
            	self.current_publisher_left.publish(self.left_current_accumulator)
	    else:
                self.current_publisher_right.publish(self.right_current_accumulator)
            
            self.current_loop_count = 0
            self.left_current_accumulator = 0.0
            self.right_current_accumulator = 0.0
        #self.current_timer = rospy.Timer(rospy.Duration(0.02), self.timer_current) # publish motor currents at 10Hz, read at 50Hz
        #self.current_publisher_left  = rospy.Publisher('left_current', Float64, queue_size=2)
        #self.current_publisher_right = rospy.Publisher('right_current', Float64, queue_size=2)
    
    def timer_odometry(self, event):
        #check for driver connected
        if self.driver is None or not self.driver.connected:
            return
        # at ~10Hz, 
        
        # poll driver for each axis position and velocity PLL estimates
        encoder_cpr = self.driver.encoder_cpr
        wheel_track = self.wheel_track   # check these. Values in m
        tyre_circumference = self.tyre_circumference
        # self.m_s_to_value = encoder_cpr/tyre_circumference set earlier
        
        # get values from ODrive
        odrive_poll_time = rospy.Time.now()
	if self.Calibrate_Axis != 1:
	    new_pos_l = self.driver.left_axis.encoder.pos_cpr    # units: encoder counts
            vel_l = self.driver.left_axis.encoder.vel_estimate  # units: encoder counts/s
	else:
            vel_r = -self.driver.right_axis.encoder.vel_estimate # neg is forward for right
            new_pos_r = -self.driver.right_axis.encoder.pos_cpr  # sign!
        
        # Twist: calculated from motor values only
	if self.Calibrate_Axis != 1:
	    vel = vel_l
	else:
	    vel = vel_r
        s = tyre_circumference * vel / (2.0*encoder_cpr)
        w = tyre_circumference * vel / (wheel_track * encoder_cpr) # angle: vel_r*tyre_radius - vel_l*tyre_radius
        self.odom_msg.twist.twist.linear.x = s
        self.odom_msg.twist.twist.angular.z = w
        
        #rospy.loginfo("vel_l: % 2.2f  vel_r: % 2.2f  vel_l: % 2.2f  vel_r: % 2.2f  x: % 2.2f  th: % 2.2f  pos_l: % 5.1f pos_r: % 5.1f " % (
        #                vel_l, -vel_r,
        #                vel_l/encoder_cpr, vel_r/encoder_cpr, self.odom_msg.twist.twist.linear.x, self.odom_msg.twist.twist.angular.z,
        #                self.driver.left_axis.encoder.pos_cpr, self.driver.right_axis.encoder.pos_cpr))
        
	if self.Calibrate_Axis != 1:
            delta_pos_l = new_pos_l - self.old_pos_l
            self.old_pos_l = new_pos_l
	else:
	    delta_pos_r = new_pos_r - self.old_pos_r
            self.old_pos_r = new_pos_r
        
        # Check for overflow. Assume we can't move more than half a circumference in a single timestep. 
        half_cpr = encoder_cpr/2.0

	if self.Calibrate_Axis != 1:
            if   delta_pos_l >  half_cpr: delta_pos_l = delta_pos_l - encoder_cpr
            elif delta_pos_l < -half_cpr: delta_pos_l = delta_pos_l + encoder_cpr
	else:
            if   delta_pos_r >  half_cpr: delta_pos_r = delta_pos_r - encoder_cpr
            elif delta_pos_r < -half_cpr: delta_pos_r = delta_pos_r + encoder_cpr
        
        # counts to metres
	if self.Calibrate_Axis != 1:
            delta_pos_l_m = delta_pos_l / self.m_s_to_value
	else:
            delta_pos_r_m = delta_pos_r / self.m_s_to_value
        
        # Distance travelled
        if self.Calibrate_Axis != 1:
	    d = delta_pos_l_m
	else:
	    d = delta_pos_r_m
        th = d/wheel_track # works for small angles
        
        xd = math.cos(th)*d
        yd = -math.sin(th)*d
        
        # elapsed time = event.last_real, event.current_real
        elapsed = (event.current_real-event.last_real).to_sec()
        # calc_vel: d/elapsed, th/elapsed
        
        # Pose: updated from previous pose + position delta
        self.x += math.cos(self.theta)*xd - math.sin(self.theta)*yd
        self.y += math.sin(self.theta)*xd + math.cos(self.theta)*yd
        self.theta = (self.theta + th) % (2*math.pi)
        
        #rospy.loginfo("dl_m: % 2.2f  dr_m: % 2.2f  d: % 2.2f  th: % 2.2f  xd: % 2.2f  yd: % 2.2f  x: % 5.1f y: % 5.1f  th: % 5.1f" % (
        #                delta_pos_l_m, delta_pos_r_m,
        #                d, th, xd, yd,
        #                self.x, self.y, self.theta
        #                ))
        
        # fill odom message and publish
        self.odom_msg.header.stamp = odrive_poll_time
        self.odom_msg.pose.pose.position.x = self.x
        self.odom_msg.pose.pose.position.y = self.y
        q = tf_conversions.transformations.quaternion_from_euler(0.0, 0.0, self.theta)
        self.odom_msg.pose.pose.orientation.z = q[2] # math.sin(self.theta)/2
        self.odom_msg.pose.pose.orientation.w = q[3] # math.cos(self.theta)/2
        
        #rospy.loginfo("theta: % 2.2f  z_m: % 2.2f  w_m: % 2.2f  q[2]: % 2.2f  q[3]: % 2.2f (q[0]: %2.2f  q[1]: %2.2f)" % (
        #                        self.theta,
        #                        math.sin(self.theta)/2, math.cos(self.theta)/2,
        #                        q[2],q[3],q[0],q[1]
        #                        ))
        
        #self.odom_msg.pose.covariance
         # x y z
         # x y z
        
        self.tf_msg.header.stamp = odrive_poll_time
        self.tf_msg.transform.translation.x = self.x
        self.tf_msg.transform.translation.y = self.y
        #self.tf_msg.transform.rotation.x
        #self.tf_msg.transform.rotation.x
        self.tf_msg.transform.rotation.z = q[2]
        self.tf_msg.transform.rotation.w = q[3]
        
        # ... and publish!
        self.odom_publisher.publish(self.odom_msg)
        if self.publish_tf:
            self.tf_publisher.sendTransform(self.tf_msg)

	if self.publish_joint_state:
	    self.state.position = new_pos_r
	    self.state.speed = vel_r
	    self.joint_state_msg.motor_temps = [0]
            self.joint_state_msg.goal_pos = self.raw_to_rad(self.state.goal, 0, False, self.RADIANS_PER_ENCODER_TICK)
            self.joint_state_msg.current_pos = self.raw_to_rad(self.state.position, 0, False, self.RADIANS_PER_ENCODER_TICK)
            self.joint_state_msg.error = 0 # state.error * self.RADIANS_PER_ENCODER_TICK
            self.joint_state_msg.velocity = self.state.speed * self.RADIANS_PER_ENCODER_TICK
            self.joint_state_msg.load = 0 # state.load
            self.joint_state_msg.is_moving = True
            self.joint_state_msg.header.stamp = rospy.Time.from_sec(rospy.Time.now().to_sec())
            self.joint_state_publisher.publish(self.joint_state_msg)
        
    def timer_check(self, event):
        """Check for cmd_vel 1 sec timeout. """
        if not self.driver:
            return
        
        if self.last_cmd_vel_time is None:
            return
        
        # if moving, and no cmd_vel received, stop
        if (event.current_real-self.last_cmd_vel_time).to_sec() > 1.0 and (self.last_speed > 0):
            rospy.logdebug("No /cmd_vel received in > 1s, stopping.")
            self.driver.drive(0,0)
            self.last_speed = 0
            self.last_cmd_vel_time = event.current_real
    
    def rad_to_raw(self, angle, initial_position_raw, flipped, encoder_ticks_per_radian):
        """ angle is in radians """
        #print 'flipped = %s, angle_in = %f, init_raw = %d' % (str(flipped), angle, initial_position_raw)
        angle_raw = angle * encoder_ticks_per_radian
        #print 'angle = %f, val = %d' % (math.degrees(angle), int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw)))
        return int(round(initial_position_raw - angle_raw if flipped else initial_position_raw + angle_raw))

    def raw_to_rad(self, raw, initial_position_raw, flipped, radians_per_encoder_tick):
        return (initial_position_raw - raw if flipped else raw - initial_position_raw) * radians_per_encoder_tick
            

def start_odrive():
    rospy.init_node('odrive')
    odrive_node = ODriveNode()
    
    rospy.spin() 
    
if __name__ == '__main__':
    try:
        start_odrive()
    except rospy.ROSInterruptException:
        pass
