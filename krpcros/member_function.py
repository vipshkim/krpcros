import rclpy
import numpy as np
from numba import njit

import krpc
from rclpy.node import Node

from std_msgs.msg import String, Header, UInt8
from geometry_msgs.msg import Pose, Twist, AccelStamped
from sensor_msgs.msg import Joy
from rosgraph_msgs.msg import Clock

import math

@njit(fastmath=True)
def calculate_gravity_ecef(grav_param, current_position, omega_vector, current_velocity):
    a_centrifugal = np.array(
            [-omega_vector[2]**2 * current_position[0], -omega_vector[2]**2 * current_position[1], 0.0])
    a_coriolis = np.array(
            [2.0 * omega_vector[2] * current_velocity[1], -2.0 * omega_vector[2] * current_velocity[0], 0.0])

    x, y, z = current_position
    r_sq = x * x + y * y + z * z
    r = np.sqrt(r_sq)
    # 방어적 프로그래밍: r이 0인 경우를 처리
    if r == 0.0:
        return np.array([0.0, 0.0, 0.0])
    factor = -grav_param / (r_sq * r)
    return np.array([factor * x, factor * y, factor * z]) - a_centrifugal + a_coriolis

def left_to_right(data):
    """
    ECEF ↔ KCKF 좌표계 간의 벡터와 사원수를 변환합니다.
    """
    return [ (item[0], item[2], item[1]) + item[3:] for item in data ]

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.conn = krpc.connect(name='krpcros', address='192.168.0.3')
        self.scene_call = self.conn.add_stream(getattr, self.conn.krpc, 'current_game_scene')
        while self.scene_call().value != 1:
            self.get_logger().info('await simulation start')
            pass
        self.init_calls()
        
        self.slow_stream_init();
        self.fast_stream_init();

        self.slow_timer = self.create_timer(0.1, self.slow_stream_callback)
        self.slow_timer = self.create_timer(0.001, self.fast_stream_callback)

        self.init_listener_callback()
        self.subscription = self.create_subscription(Joy, 'krpcros/joy', self.joy_listener_callback,10)
        self.subscription

    def init_listener_callback(self):
        self.msg_joy = Joy()
        self.msg_joy.buttons = (0,0,0,0,0,0,0,0,0)
        self.axis_names = [f"custom_axis{i:02d}" for i in range(1, 4)]
    
    def joy_listener_callback(self, msg):
        control = self.vessel.control
        for i in range(len(msg.axis)):
            if i > 4:
                break
            setattr(control,self.axis_names[i],msg.axis[i])
        for j in range(len(msg.button)):
            if j > 9:
                break
            if self.msg_joy.buttons[j] != msg.buttons[j]:
                self.msg_joy.buttions[j] = msg.buttons[j]
                control.set_action_group(j,msg.buttons[j])

    def init_calls(self):
        self.vessel = self.conn.space_center.active_vessel
        self.body = self.vessel.orbit.body
        self.root = self.vessel.parts.root

        eciframe = self.body.non_rotating_reference_frame
        ecefframe = self.body.reference_frame
        surfframe = self.vessel.surface_reference_frame
        self.position_call = self.conn.add_stream(self.root.position, ecefframe)
        self.position_surf_call = self.conn.add_stream(self.root.position, surfframe)
        self.orientation_call = self.conn.add_stream(self.root.rotation, ecefframe)
        self.orientation_surf_call = self.conn.add_stream(self.root.rotation, surfframe)
        self.velocity_call = self.conn.add_stream(self.root.velocity, ecefframe)
        self.angvel_call = self.conn.add_stream(self.root.vessel.angular_velocity, ecefframe)
        self.grav_param_call = self.conn.add_stream(getattr, self.body, 'gravitational_parameter')
        self.situation_call = self.conn.add_stream(getattr, self.vessel, 'situation',)
        self.stamp_call = self.conn.add_stream(getattr, self.conn.space_center, 'ut')

    def slow_stream_init(self):
        self.body_rotational_speed_call = self.conn.add_stream(getattr, self.body, 'rotational_speed')
        self.omg_scr = self.body_rotational_speed_call()
        self.omg_vec = np.array([0.0,0.0,self.omg_scr])

        self.debug_pub = self.create_publisher(String, 'krpcros/debug', 10)

        self.grav_param = self.grav_param_call()
        self.omg_scr = self.body_rotational_speed_call()
        self.omg_vec = np.array([0.0,0.0,self.omg_scr])

    def slow_stream_callback(self):
        #각속도
        self.grav_param = self.grav_param_call()
        self.omg_scr = self.body_rotational_speed_call()
        self.omg_vec = np.array([0.0,0.0,self.omg_scr])

        self.situ_msg = UInt8()
        self.situ_msg.data = (self.scene_call().value<<4 & 0xf0);
        if self.scene_call().value == 1:
            self.situ_msg.data |= (self.situation_call().value & 0xf)

        #publisher
        msg=String()
        msg.data = "updating"

        self.debug_pub.publish(msg)
        self.situ_pub.publish(self.situ_msg)


    def fast_stream_init(self):
        #krpc stream
        self.stamp = self.stamp_call()

        self.rate_msg = Twist()
        current_velocity, current_angvel = left_to_right([self.velocity_call(),
                                            self.angvel_call() ])
        self.rate_msg.linear.x = current_velocity[0]
        self.rate_msg.linear.y = current_velocity[1]
        self.rate_msg.linear.z = current_velocity[2]
        self.rate_msg.angular.x = -current_angvel[0]
        self.rate_msg.angular.y = -current_angvel[1]
        self.rate_msg.angular.z = -current_angvel[2]

        #ros stream
        self.pose_surf_pub_raw = self.create_publisher(Pose, 'krpcros/pose_surf_raw', 10)
        self.pose_pub_raw   = self.create_publisher(Pose, 'krpcros/pose_raw', 10)
        self.rate_pub_raw   = self.create_publisher(Twist, 'krpcros/rate_raw', 10)
        self.accel_pub      = self.create_publisher(AccelStamped, 'krpcros/accel', 10)
        self.situ_pub       = self.create_publisher(UInt8, 'krpcros/situation', 10)
        self.clock_pub      = self.create_publisher(Clock, '/clock', 10)

        self.header = Header()
        self.pose_msg = Pose()
        self.pose_surf_msg = Pose()
        self.rate_msg = Twist()
        self.accel_msg = AccelStamped()
        self.clock_msg = Clock()
        self.scene = self.scene_call().value

    def fast_stream_callback(self):
        if self.scene != self.scene_call().value:
            self.scene = self.scene_call().value
            if self.scene == 1:
                self.init_calls()
        if self.scene != 1:
            return
        current_stamp = self.stamp_call();
        dt = current_stamp-self.stamp;
        if dt==0:
            return;
        self.stamp = current_stamp

        #timestamp
        header = self.header
        pose_msg = self.pose_msg
        pose_surf_msg = self.pose_surf_msg
        rate_msg = self.rate_msg
        accel_msg = self.accel_msg
        clock_msg = self.clock_msg


        sec, frac = divmod(current_stamp,1)
        header.stamp.sec = int(sec)
        header.stamp.nanosec = int(frac*1e9)

        #Position
        current_position, current_velocity, current_angvel, current_orientation,\
        current_surf_position, current_surf_orientation \
        = left_to_right([ self.position_call(),
                          self.velocity_call(),
                          self.angvel_call(),
                          self.orientation_call(),
                          self.position_surf_call(),
                          self.orientation_surf_call(),
                         ])

        pose_msg.position.x = current_position[0]
        pose_msg.position.y = current_position[1]
        pose_msg.position.z = current_position[2]
        pose_msg.orientation.x = current_orientation[0]
        pose_msg.orientation.y = current_orientation[1]
        pose_msg.orientation.z = current_orientation[2]
        pose_msg.orientation.w = current_orientation[3]

        pose_surf_msg.position.x = current_surf_position[0]
        pose_surf_msg.position.y = current_surf_position[1]
        pose_surf_msg.position.z = current_surf_position[2]
        pose_surf_msg.orientation.x = current_surf_orientation[0]
        pose_surf_msg.orientation.y = current_surf_orientation[1]
        pose_surf_msg.orientation.z = current_surf_orientation[2]
        pose_surf_msg.orientation.w = current_surf_orientation[3]
        
        cv_x = current_velocity[0]
        cv_y = current_velocity[1]
        cv_z = current_velocity[2]
        ca_x = -current_angvel[0]
        ca_y = -current_angvel[1]
        ca_z = -current_angvel[2]
        
        delta_velocity_x = cv_x - rate_msg.linear.x
        delta_velocity_y = cv_y - rate_msg.linear.y
        delta_velocity_z = cv_z - rate_msg.linear.z
        delta_angvel_x = ca_x - rate_msg.angular.x
        delta_angvel_y = ca_y - rate_msg.angular.y
        delta_angvel_z = ca_z - rate_msg.angular.z

        rate_msg.linear.x = cv_x
        rate_msg.linear.y = cv_y
        rate_msg.linear.z = cv_z
        rate_msg.angular.x = ca_x
        rate_msg.angular.y = ca_y
        rate_msg.angular.z = ca_z
        
        accel_msg.header = header;

        ground_accel = calculate_gravity_ecef(self.grav_param, 
                                            current_position,
                                            self.omg_vec,
                                            current_velocity )

        x= accel_msg.accel.linear.x = delta_velocity_x/dt - ground_accel[0]
        y= accel_msg.accel.linear.y = delta_velocity_y/dt - ground_accel[1]
        z= accel_msg.accel.linear.z = delta_velocity_z/dt - ground_accel[2]
        accel_msg.accel.angular.x = delta_angvel_x/dt
        accel_msg.accel.angular.y = delta_angvel_y/dt
        accel_msg.accel.angular.z = delta_angvel_z/dt

        #simulation clock
        clock_msg.clock = header.stamp

        #publish
        self.clock_pub.publish(clock_msg)
        self.pose_surf_pub_raw.publish(pose_surf_msg)
        self.pose_pub_raw.publish(pose_msg)
        self.rate_pub_raw.publish(rate_msg)
        self.accel_pub.publish(accel_msg)


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()

    try:
        rclpy.spin(minimal_publisher)
    except KeyboardInterrupt:
        pass

    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
