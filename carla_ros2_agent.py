"""
This module provides a bridge between Carla ROS and Autoware Universe
"""
import os
import subprocess
import signal
import threading
import time
import rclpy
import math
import numpy
# from rclpy.time import Time
from rclpy.qos import QoSProfile
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from cv_bridge import CvBridge

from geometry_msgs.msg import TwistWithCovarianceStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, PointCloud2, NavSatFix, NavSatStatus, CameraInfo, Imu
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from autoware_auto_vehicle_msgs.msg import ControlModeReport, GearReport, SteeringReport, TurnIndicatorsReport, HazardLightsReport, VelocityReport, TurnIndicatorsCommand, HazardLightsCommand
from autoware_auto_control_msgs.msg import AckermannControlCommand
from autoware_sensing_msgs.msg import GnssInsOrientationStamped
from tier4_vehicle_msgs.msg import ActuationStatusStamped

from ackermann_msgs.msg import AckermannDrive
from carla_msgs.msg import *
from carla_ackermann_msgs.msg import *
from std_msgs.msg import Float32

def get_entry_point():
    return 'Ros2Agent'

class Ros2Agent:
    stack_process = None    
    start_script = None

    carla_max_steer_angle = None

    def __init__(self, world_handler):
        """
        setup agent
        """
        self.agent_role_name = os.environ['AGENT_ROLE_NAME']
        self.topic_base = "/carla/{}".format(self.agent_role_name)

        self.world_handler = world_handler
        self.town_map_name = self._get_map_name(world_handler.get_world().get_map().name)
    
        self.cv_bridge = CvBridge()

        self.carla_current_vehicle_info = None
        self.carla_current_vehicle_status = None
        self.current_brake_light = False
        self.current_reverse_light = False

        # get start_script from environment
        script_code_path = os.environ['SCRIPT_ROOT']
        if not script_code_path or not os.path.exists(script_code_path):
            raise IOError("Path '{}' defined by SCRIPT_ROOT invalid".format(script_code_path))
        self.start_script = "{}/start_ros2.sh".format(script_code_path)
        if not os.path.exists(self.start_script):
            raise IOError("File '{}' defined by SCRIPT_ROOT invalid".format(self.start_script))

        # initialize ros2 node
        rclpy.init(args=None)
        self.ros2_node = rclpy.create_node("op_ros2_agent")
        self.ros2_executor = MultiThreadedExecutor()
        self.ros2_callback_group = ReentrantCallbackGroup()

        # Carla Ackermann controls
        self.carla_ackermann_pub = self.ros2_node.create_publisher(
                    AckermannDrive, "{}/ackermann_cmd".format(self.topic_base), 10, callback_group=self.ros2_callback_group)

        # Carla Lights Control
        self.carla_light_pub = self.ros2_node.create_publisher(
            CarlaEgoVehicleLight, "{}/vehicle_light_cmd".format(self.topic_base), 10, callback_group=self.ros2_callback_group)

        # Carla Lidar repub
        self.carla_lidar_sub = self.ros2_node.create_subscription(
            PointCloud2, "{}/lidar".format(self.topic_base), self.on_lidar_recieve, 1, callback_group=self.ros2_callback_group)
        self.carla_lidar_pub_loc = self.ros2_node.create_publisher(
                    PointCloud2, '/sensing/lidar/top/outlier_filtered/pointcloud', 10, callback_group=self.ros2_callback_group)
        self.carla_lidar_pub_per = self.ros2_node.create_publisher(
                    PointCloud2, '/sensing/lidar/concatenated/pointcloud', 10, callback_group=self.ros2_callback_group)

        # Carla GNSS repub
        self.carla_gnss_sub = self.ros2_node.create_subscription(
            NavSatFix, "{}/gnss".format(self.topic_base), self.on_gnss_recieve, 1, callback_group=self.ros2_callback_group)
        self.carla_gnss_pub = self.ros2_node.create_publisher(
                    NavSatFix,  "/sensing/gnss/ublox/nav_sat_fix", 1, callback_group=self.ros2_callback_group)

        # Carla IMU repub
        self.carla_gnss_sub = self.ros2_node.create_subscription(
            Imu, "{}/imu".format(self.topic_base), self.on_imu_recieve, 1, callback_group=self.ros2_callback_group)
        self.carla_imu_pub = self.ros2_node.create_publisher(
                        Imu, '/sensing/imu/tamagawa/imu_raw', 1, callback_group=self.ros2_callback_group)
        
        # Carla Twist Publisher
        self.carla_twist_pub = self.ros2_node.create_publisher(
                        TwistWithCovarianceStamped, '/sensing/vehicle_velocity_converter/twist_with_covariance', 1, callback_group=self.ros2_callback_group)

        # Carla Vehicle Info
        self.carla_vehicleinfo_sub = self.ros2_node.create_subscription(
            CarlaEgoVehicleInfo, "{}/vehicle_info".format(self.topic_base), self.on_vehicleinfo_recieve, 1, callback_group=self.ros2_callback_group)

        # Carla Vehicle Status
        self.carla_vehiclestatus_sub = self.ros2_node.create_subscription(
            CarlaEgoVehicleStatus, "{}/vehicle_status".format(self.topic_base), self.on_vehiclestatus_recieve, 1, callback_group=self.ros2_callback_group)

        # Carla Speedometer
        self.carla_speedometer_sub = self.ros2_node.create_subscription(
            Float32, "{}/speedometer".format(self.topic_base), self.on_speedometer_recieve, 1, callback_group=self.ros2_callback_group)

        # Carla Camera
        self.carla_front_image_sub = self.ros2_node.create_subscription(
            Image, "{}/rgb_front/image".format(self.topic_base), self.on_rgb_front_image_recieve, 1, callback_group=self.ros2_callback_group)
        self.front_camera_info_sub = self.ros2_node.create_subscription(
            CameraInfo, "{}/rgb_front/camera_info".format(self.topic_base), self.on_rgb_front_camera_info_recieve, 1, callback_group=self.ros2_callback_group)

        self.rgb_front_image_pub = self.ros2_node.create_publisher(
                    Image, "/sensing/camera/camera0/image_rect_color", 1, callback_group=self.ros2_callback_group)
        self.rgb_front_camera_info_pub = self.ros2_node.create_publisher(
                    CameraInfo, "/sensing/camera/camera0/camera_info", 1, callback_group=self.ros2_callback_group)

        # Odometry publisher
        # self.vehicle_odo_pub = self.ros2_node.create_publisher(
        #     Odometry, '/odo', 1, callback_group=self.ros2_callback_group)

        self.vehicle_gnss_ins_pub = self.ros2_node.create_publisher(
            GnssInsOrientationStamped, '/autoware_orientation', 1, callback_group=self.ros2_callback_group)
        
        # self.vehicle_odo_sub = self.ros2_node.create_subscription(
        #     Odometry, "{}/odometry".format(self.topic_base), self.on_odometry_recieve, 1, callback_group=self.ros2_callback_group)

        self.auto_velocity_status_publisher = self.ros2_node.create_publisher(
            VelocityReport, '/vehicle/status/velocity_status', 1, callback_group=self.ros2_callback_group)

        self.auto_steering_status_publisher = self.ros2_node.create_publisher(
            SteeringReport, '/vehicle/status/steering_status', 1, callback_group=self.ros2_callback_group)

        self.auto_gear_status_publisher = self.ros2_node.create_publisher(
            GearReport, '/vehicle/status/gear_status', 1, callback_group=self.ros2_callback_group)

        self.auto_turning_status_publisher = self.ros2_node.create_publisher(
            TurnIndicatorsReport, '/vehicle/status/turn_indicators_status', 1, callback_group=self.ros2_callback_group)

        self.auto_hazard_status_publisher = self.ros2_node.create_publisher(
            HazardLightsReport, '/vehicle/status/hazard_lights_status', 1, callback_group=self.ros2_callback_group)

        self.auto_control_mode_publisher = self.ros2_node.create_publisher(
            ControlModeReport, '/vehicle/status/control_mode', 1, callback_group=self.ros2_callback_group)

        self.actuation_status_publisher = self.ros2_node.create_publisher(
            ActuationStatusStamped, '/vehicle/status/actuation_status', 1, callback_group=self.ros2_callback_group)

        self.autoware_universe_vehicle_control_subscriber = self.ros2_node.create_subscription(
            AckermannControlCommand, '/control/command/control_cmd', self.on_autoware_universe_vehicle_control,
            qos_profile=QoSProfile(depth=1), callback_group=self.ros2_callback_group)

        self.carla_ego_vehicle_lights_subscriber = self.ros2_node.create_subscription(
            CarlaEgoVehicleControl, "{}/vehicle_control_cmd".format(self.topic_base), self.on_carla_vehicle_lights_control,
            qos_profile=QoSProfile(depth=1), callback_group=self.ros2_callback_group)

        self.autoware_universe_vehicle_turn_indicator_subscriber = self.ros2_node.create_subscription(
            TurnIndicatorsCommand, '/control/command/turn_indicators_cmd', self.on_autoware_universe_vehicle_turn_lights,
            qos_profile=QoSProfile(depth=1), callback_group=self.ros2_callback_group)

        self.autoware_universe_vehicle_control_subscriber = self.ros2_node.create_subscription(
            HazardLightsCommand, '/control/command/hazard_lights_cmd', self.on_autoware_universe_vehicle_hazard_lights,
            qos_profile=QoSProfile(depth=1), callback_group=self.ros2_callback_group)
        
        self.spin_thread = threading.Thread(target=rclpy.spin, args=(self.ros2_node, self.ros2_executor))
        self.spin_thread.start()

    def on_lidar_recieve(self, data):
        data.header.frame_id = 'velodyne_top'
        self.carla_lidar_pub_loc.publish(data)
        self.carla_lidar_pub_per.publish(data)

    def on_gnss_recieve(self, data): 
        data.header.frame_id = 'gnss_link'
        data.status.status = NavSatStatus.STATUS_SBAS_FIX
        # pylint: disable=line-too-long
        data.status.service = NavSatStatus.SERVICE_GPS | NavSatStatus.SERVICE_GLONASS | NavSatStatus.SERVICE_COMPASS | NavSatStatus.SERVICE_GALILEO
        # pylint: enable=line-too-long

        self.carla_gnss_pub.publish(data)

    def on_imu_recieve(self, data): 
        data.header.frame_id = "tamagawa/imu_link"
        self.carla_imu_pub.publish(data)

        gnss_ins = GnssInsOrientationStamped()
        gnss_ins.header = self.get_header()
        gnss_ins.header.frame_id = "gnss_link"

        gnss_ins.orientation.rmse_rotation_x = 0.1
        gnss_ins.orientation.rmse_rotation_y = 0.1
        gnss_ins.orientation.rmse_rotation_z = 1.0
        gnss_ins.orientation.orientation = data.orientation

        self.vehicle_gnss_ins_pub.publish(gnss_ins)

        if self.carla_current_vehicle_status is None:
            return

        twistStamped = TwistWithCovarianceStamped()
        stddev_vx_ = 0.04 # 0.2 * 0.2 for std dev convariance
        stddev_wz_ = 0.01 # 0.1 * 0.1 for std dev convariance
        
        twistStamped.header = self.get_header()
        twistStamped.header.frame_id = "base_link"
        
        twistStamped.twist.twist.angular = data.angular_velocity
        twistStamped.twist.twist.linear.x = self.carla_current_vehicle_status.velocity
        
        twistStamped.twist.covariance = numpy.ndarray(36, dtype=numpy.float64)
        twistStamped.twist.covariance[0] = stddev_vx_
        twistStamped.twist.covariance[7] = 10000.0
        twistStamped.twist.covariance[14] = 10000.0
        twistStamped.twist.covariance[21] = 10000.0
        twistStamped.twist.covariance[28] = 10000.0
        twistStamped.twist.covariance[35] = stddev_wz_

        self.carla_twist_pub.publish(twistStamped)


    def on_vehicleinfo_recieve(self, data):
        # Carla info return to be processes as needed
        self.carla_current_vehicle_info = data

        self.carla_max_steer_angle = abs(data.wheels[0].max_steer_angle)

    def on_vehiclestatus_recieve(self, data):
        # Carla status return to be processes as needed
        self.carla_current_vehicle_status = data

        steer_rep = SteeringReport()
        steer_rep.steering_tire_angle = math.radians(-data.wheel_steer_angle)
        steer_rep.stamp = self.get_timestamp()
        self.auto_steering_status_publisher.publish(steer_rep)

        # Turn and hazard status reporting
        turn_rep = TurnIndicatorsReport()
        hazard_rep = HazardLightsReport()
        hazard_rep.stamp = self.get_timestamp()
        turn_rep.stamp = self.get_timestamp()
        
        if data.light.hazard:
            hazard_rep.report = HazardLightsReport.ENABLE
            turn_rep.report = TurnIndicatorsReport.DISABLE
        elif data.light.right_blinker:
            hazard_rep.report = HazardLightsReport.DISABLE
            turn_rep.report = TurnIndicatorsReport.ENABLE_RIGHT
        elif data.light.left_blinker:
            hazard_rep.report = HazardLightsReport.DISABLE
            turn_rep.report = TurnIndicatorsReport.ENABLE_LEFT
        else: 
            hazard_rep.report = HazardLightsReport.DISABLE
            turn_rep.report = TurnIndicatorsReport.DISABLE
        
        self.auto_turning_status_publisher.publish(turn_rep)
        self.auto_hazard_status_publisher.publish(hazard_rep)

        gear_rep = GearReport()
        if data.control.reverse == True:
            gear_rep.report = GearReport.REVERSE
        elif data.control.gear == 3:
            gear_rep.report = GearReport.DRIVE
        elif data.control.gear == 0 and data.control.hand_brake == True:
            gear_rep.report = GearReport.PARK
        elif data.control.gear == 0:
            gear_rep.report = GearReport.NEUTRAL
        elif data.control.gear == 1:
            gear_rep.report = GearReport.DRIVE
        else: # Unknown value so report as none
            gear_rep.report = GearReport.NONE
        gear_rep.stamp = self.get_timestamp()
        self.auto_gear_status_publisher.publish(gear_rep)

        control_mode_rep = ControlModeReport()
        control_mode_rep.mode = ControlModeReport.AUTONOMOUS
        control_mode_rep.stamp = self.get_timestamp()
        self.auto_control_mode_publisher.publish(control_mode_rep)   

        actualtion_status = ActuationStatusStamped()
        actualtion_status.header = self.get_header()
        actualtion_status.header.frame_id = "base_link"
        actualtion_status.status.accel_status = data.control.throttle
        actualtion_status.status.brake_status = data.control.brake
        actualtion_status.status.steer_status = data.control.steer
        self.actuation_status_publisher.publish(actualtion_status)

    def on_speedometer_recieve(self, data):
        vel_rep = VelocityReport()
        vel_rep.header = self.get_header()
        vel_rep.header.frame_id = "base_link"
        vel_rep.longitudinal_velocity = data.data;                                 
        vel_rep.heading_rate = 0.0
        self.auto_velocity_status_publisher.publish(vel_rep)

    def on_rgb_front_image_recieve(self, data):
        data.header.frame_id = 'camera0/camera_link'
        self.rgb_front_image_pub.publish(data)

    def on_rgb_front_camera_info_recieve(self, data):
        data.header.frame_id = 'camera0/camera_link'
        self.rgb_front_camera_info_pub.publish(data)

    def on_odometry_recieve(self, data):
        # Not currently needed, gnss_ins moved to imu
        pass
        # gnss_ins = GnssInsOrientationStamped()
        # gnss_ins.header = self.get_header()
        # gnss_ins.header.frame_id = "gnss_link"

        # gnss_ins.orientation.rmse_rotation_x = 0.1
        # gnss_ins.orientation.rmse_rotation_y = 0.1
        # gnss_ins.orientation.rmse_rotation_z = 1.0
        # gnss_ins.orientation.orientation.x = 0.0
        # gnss_ins.orientation.orientation.y = 0.0
        # gnss_ins.orientation.orientation.z = data.pose.pose.orientation.z
        # gnss_ins.orientation.orientation.w = data.pose.pose.orientation.w

        # self.vehicle_gnss_ins_pub.publish(gnss_ins)
        # self.vehicle_odo_pub.publish(data)

    def init_local_agent(self, role_name, map_name, waypoints_topic_name, enable_explore):
        carla_ip = os.environ['SIMULATOR_LOCAL_HOST']
        carla_port = os.environ['SIMULATOR_PORT']

        # rospy.loginfo("Executing stack...")
        print("Executing stack...", role_name, map_name)
        local_start_script = self.start_script + ' ' + role_name + ' ' + map_name + ' ' + enable_explore + ' ' + carla_ip + ' ' + carla_port + ' ' + waypoints_topic_name
        self.stack_process = subprocess.Popen(local_start_script, shell=True, preexec_fn=os.setpgrp)
        # self.vehicle_control_event = threading.Event()

    def on_autoware_universe_vehicle_control(self, data):
        control = AckermannDrive()

        control.speed = data.longitudinal.speed
        control.acceleration = data.longitudinal.acceleration
        control.jerk = data.longitudinal.jerk

        control.steering_angle = data.lateral.steering_tire_angle
        control.steering_angle_velocity = data.lateral.steering_tire_rotation_rate

        self.carla_ackermann_pub.publish(control)


    def on_carla_vehicle_lights_control(self, data):
        lights = CarlaEgoVehicleLight()
        if (data.brake > 0):
            lights.brake = CarlaEgoVehicleLight.ON
        else: 
            lights.brake = CarlaEgoVehicleLight.OFF

        if data.reverse:
            lights.reverse = CarlaEgoVehicleLight.ON
        else:
            lights.reverse = CarlaEgoVehicleLight.OFF

        self.carla_light_pub.publish(lights)

    def on_autoware_universe_vehicle_turn_lights(self, data):
        lights = CarlaEgoVehicleLight()

        if data.command == TurnIndicatorsCommand.NO_COMMAND:
            return
        elif data.command == TurnIndicatorsCommand.DISABLE:
            lights.left_blinker = CarlaEgoVehicleLight.OFF
            lights.right_blinker = CarlaEgoVehicleLight.OFF
        elif data.command == TurnIndicatorsCommand.ENABLE_LEFT:
            lights.left_blinker = CarlaEgoVehicleLight.ON
            lights.right_blinker = CarlaEgoVehicleLight.OFF
        elif data.command == TurnIndicatorsCommand.ENABLE_RIGHT:
            lights.left_blinker = CarlaEgoVehicleLight.OFF
            lights.right_blinker = CarlaEgoVehicleLight.ON
        else:
            return
            # raise RuntimeError("Unknown Turn Indicator Command")

        self.carla_light_pub.publish(lights)

    def on_autoware_universe_vehicle_hazard_lights(self, data):
        lights = CarlaEgoVehicleLight()

        if data.command == HazardLightsCommand.NO_COMMAND:
            return
        elif data.command == HazardLightsCommand.DISABLE:
            lights.hazard = CarlaEgoVehicleLight.OFF
        elif data.command == HazardLightsCommand.ENABLE:
            lights.hazard = CarlaEgoVehicleLight.ON
        else:
            return
            # raise RuntimeError("Unknown Hazard Indicator Command")

        self.carla_light_pub.publish(lights)

    def get_header(self):
        """
        Returns ROS message header
        """
        header = Header()
        header.stamp = self.get_timestamp() 

        #print('Sensor Time Stamp: ', header.stamp)    
        return header

    def get_timestamp(self):
        return self.header_timestamp

    def run_step(self, timestamp):
        """
        Execute one step of navigation.
        """        
        
        self.timestamp = timestamp

        # Make timestamp once as no need to remake each time
        seconds = int(self.timestamp.elapsed_seconds)
        nanoseconds = int((self.timestamp.elapsed_seconds - int(self.timestamp.elapsed_seconds)) * 1000000000.0)
        self.header_timestamp = Time(sec=seconds, nanosec=nanoseconds)    

        if self.stack_process is None and self.town_map_name is not None:
            self.init_local_agent(self.agent_role_name, self.town_map_name, '', 'true')
            time.sleep(30.0)
        
            # Send a stop command to the ackermann control to make sure the spawned vehicle is stopped
            control = AckermannDrive()
            control.speed = 0.0
            control.acceleration = 0.0
            control.jerk = 0.0
            control.steering_angle = 0.0
            control.steering_angle_velocity = 0.0
            self.carla_ackermann_pub.publish(control)

        # check if stack is still running
        if self.stack_process and self.stack_process.poll() is not None:
            self.running = False
            raise RuntimeError("Stack exited with: {} {}".format(
                self.stack_process.returncode, self.stack_process.communicate()[0]))
        
        # Check if spin thread is still alive
        if self.spin_thread.is_alive() is not True:
            self.running = False
            raise RuntimeError("Spinthread is no longer alive")

        if self.carla_current_vehicle_info != None and self.carla_current_vehicle_status != None:
            return {
                "info": self.carla_current_vehicle_info,
                "status": self.carla_current_vehicle_status
            }
        else:
            return None

    def destroy(self):
        """
        Cleanup of all ROS publishers
        """        
        if self.stack_process and self.stack_process.poll() is None:
            self.ros2_node.get_logger().info("Sending SIGTERM to stack...")
            os.killpg(os.getpgid(self.stack_process.pid), signal.SIGTERM)
            self.ros2_node.get_logger().info("Waiting for termination of stack...")
            self.stack_process.wait()
            time.sleep(5)
            self.ros2_node.get_logger().info("Terminated stack.")

        self.ros2_node.get_logger().info("Stack is no longer running")        

        if self.stack_process:
            self.stack_process = None

        time.sleep(5)

        self.ros2_node.destroy_node()
        rclpy.shutdown()

        # raise TypeError("Just Stop ................. Please ")
        print("Cleanup finished")

    def _get_map_name(self, map_full_name):

        if map_full_name is None:
            return None
        name_start_index = map_full_name.rfind("/")
        if name_start_index == -1:
            name_start_index = 0
        else:
            name_start_index = name_start_index + 1        

        return map_full_name[name_start_index:len(map_full_name)]