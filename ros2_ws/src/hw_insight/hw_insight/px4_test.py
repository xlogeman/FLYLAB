#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleLocalPosition, VehicleStatus
from px4_msgs.msg import ActuatorMotors, ActuatorServos, ArmingCheckReply, VehicleGlobalPosition, VehicleControlMode
from px4_msgs.msg import ConfigOverrides, DifferentialDriveSetpoint, GotoSetpoint, ManualControlSetpoint
from px4_msgs.msg import MessageFormatRequest, ModeCompleted, ObstacleDistance
from px4_msgs.msg import OnboardComputerStatus, RegisterExtComponentRequest, SensorOpticalFlow, TelemetryStatus
from px4_msgs.msg import  UnregisterExtComponent, VehicleAttitudeSetpoint
from px4_msgs.msg import VehicleRatesSetpoint, VehicleThrustSetpoint, VehicleTorqueSetpoint
from px4_msgs.msg import VehicleTrajectoryBezier, VehicleTrajectoryWaypoint, VehicleOdometry
from px4_msgs.msg import RegisterExtComponentReply, AirspeedValidated, ArmingCheckRequest
from px4_msgs.msg import BatteryStatus, CollisionConstraints, EstimatorStatusFlags, FailsafeFlags
from px4_msgs.msg import MessageFormatResponse, Mission, MissionResult
from px4_msgs.msg import PositionSetpointTriplet, SensorCombined, TimesyncStatus
from px4_msgs.msg import VehicleCommandAck, VehicleAttitude, SensorGps

class px4_test_def(Node):
    def __init__(self) -> None:
        super().__init__('px4_test')

        # Configure QoS profile for publishing and subscribing
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # 创建PX4相关话题的发布者和订阅者
        self.create_all_px4_publishers()
        self.create_all_px4_subscribers()

        # 创建定时器，定时显示所有订阅信息
        self.timer_print_subscribe = self.create_timer(1, self.timer_print_subscribe_callback)

    # 创建所有的PX4相关话题的发布者 ----------------------------------------------------------------------------------------------------------------------------------
    def create_all_px4_publishers(self):
        #topic: /fmu/in/actuator_motors
        #type:  px4_msgs::msg::ActuatorMotors
        self.px4_fmu_in_actuator_motors_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', self.qos_profile)

        # topic: /fmu/in/actuator_servos
        # type:  px4_msgs::msg::ActuatorServos
        self.px4_fmu_in_actuator_servos_publisher = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', self.qos_profile)

        # topic: /fmu/in/arming_check_reply
        # type:  px4_msgs::msg::ArmingCheckReply
        self.px4_fmu_in_arming_check_reply_publisher = self.create_publisher(ArmingCheckReply, '/fmu/in/arming_check_reply', self.qos_profile)

        # topic: /fmu/in/aux_global_position
        # type:  px4_msgs::msg::VehicleGlobalPosition
        self.px4_fmu_in_aux_global_position_publisher = self.create_publisher(VehicleGlobalPosition, '/fmu/in/aux_global_position', self.qos_profile)

        # topic: /fmu/in/config_control_setpoints
        # type:  px4_msgs::msg::VehicleControlMode
        self.px4_fmu_in_config_control_setpoints_publisher = self.create_publisher(VehicleControlMode, '/fmu/in/config_control_setpoints', self.qos_profile)

        # topic: /fmu/in/config_overrides_request
        # type:  px4_msgs::msg::ConfigOverrides
        self.px4_fmu_in_config_overrides_request_publisher = self.create_publisher(ConfigOverrides, '/fmu/in/config_overrides_request', self.qos_profile)

        # topic: /fmu/in/differential_drive_setpoint
        # type: px4_msgs::msg::DifferentialDriveSetpoint
        self.px4_fmu_in_differential_drive_setpoint_publisher = self.create_publisher(DifferentialDriveSetpoint, '/fmu/in/differential_drive_setpoint', self.qos_profile)
        
        # topic: /fmu/in/goto_setpoint
        # type: px4_msgs::msg::GotoSetpoint
        self.px4_fmu_in_goto_setpoint_publisher = self.create_publisher(GotoSetpoint, '/fmu/in/goto_setpoint', self.qos_profile)
        
        # topic: /fmu/in/manual_control_input
        # type: px4_msgs::msg::ManualControlSetpoint
        self.px4_fmu_in_manual_control_input_publisher = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', self.qos_profile)
        
        # topic: /fmu/in/message_format_request
        # type: px4_msgs::msg::MessageFormatRequest
        self.px4_fmu_in_message_format_request_publisher = self.create_publisher(MessageFormatRequest, '/fmu/in/message_format_request', self.qos_profile)
       
        # topic: /fmu/in/mode_completed
        # type: px4_msgs::msg::ModeCompleted
        self.px4_fmu_in_mode_completed_publisher = self.create_publisher(ModeCompleted, '/fmu/in/mode_completed', self.qos_profile)
        
        # topic: /fmu/in/obstacle_distance
        # type: px4_msgs::msg::ObstacleDistance
        self.px4_fmu_in_obstacle_distance_publisher = self.create_publisher(ObstacleDistance, '/fmu/in/obstacle_distance', self.qos_profile)
        
        # topic: /fmu/in/offboard_control_mode
        # type: px4_msgs::msg::OffboardControlMode
        self.px4_fmu_in_offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.qos_profile)
        
        # topic: /fmu/in/onboard_computer_status
        # type: px4_msgs::msg::OnboardComputerStatus
        self.px4_fmu_in_onboard_computer_status_publisher = self.create_publisher(OnboardComputerStatus, '/fmu/in/onboard_computer_status', self.qos_profile)
        
        # topic: /fmu/in/register_ext_component_request
        # type: px4_msgs::msg::RegisterExtComponentRequest
        self.px4_fmu_in_register_ext_component_request_publisher = self.create_publisher(RegisterExtComponentRequest, '/fmu/in/register_ext_component_request', self.qos_profile)
        
        # topic: /fmu/in/sensor_optical_flow
        # type: px4_msgs::msg::SensorOpticalFlow
        self.px4_fmu_in_sensor_optical_flow_publisher = self.create_publisher(SensorOpticalFlow, '/fmu/in/sensor_optical_flow', self.qos_profile)
        
        # topic: /fmu/in/telemetry_status
        # type: px4_msgs::msg::TelemetryStatus
        self.px4_fmu_in_telemetry_status_publisher = self.create_publisher(TelemetryStatus, '/fmu/in/telemetry_status', self.qos_profile)
        
        # topic: /fmu/in/trajectory_setpoint
        # type: px4_msgs::msg::TrajectorySetpoint
        self.px4_fmu_in_trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.qos_profile)
        
        # topic: /fmu/in/unregister_ext_component
        # type: px4_msgs::msg::UnregisterExtComponent
        self.px4_fmu_in_unregister_ext_component_publisher = self.create_publisher(UnregisterExtComponent, '/fmu/in/unregister_ext_component', self.qos_profile)
        
        # topic: /fmu/in/vehicle_attitude_setpoint
        # type: px4_msgs::msg::VehicleAttitudeSetpoint
        self.px4_fmu_in_vehicle_attitude_setpoint_Pubvehicle_attitude_setpointlisher_publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', self.qos_profile)
        
        # topic: /fmu/in/vehicle_command
        # type: px4_msgs::msg::VehicleCommand
        self.px4_fmu_in_vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', self.qos_profile)
        
        # topic: /fmu/in/vehicle_command_mode_executor
        # type: px4_msgs::msg::VehicleCommand
        self.px4_fmu_in_vehicle_command_mode_executor_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_mode_executor', self.qos_profile)
        
        # topic: /fmu/in/vehicle_mocap_odometry
        # type: px4_msgs::msg::VehicleOdometry
        self.px4_fmu_in_vehicle_mocap_odometry_publisher = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_mocap_odometry', self.qos_profile)
        
        # topic: /fmu/in/vehicle_rates_setpoint
        # type: px4_msgs::msg::VehicleRatesSetpoint
        self.px4_fmu_in_vehicle_rates_setpoint_publisher = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', self.qos_profile)
        
        # topic: /fmu/in/vehicle_thrust_setpoint
        # type: px4_msgs::msg::VehicleThrustSetpoint
        self.px4_fmu_in_vehicle_thrust_setpoint_publisher = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', self.qos_profile)
        
        # topic: /fmu/in/vehicle_torque_setpoint
        # type: px4_msgs::msg::VehicleTorqueSetpoint
        self.px4_fmu_in_vehicle_torque_setpoint_publisher = self.create_publisher(VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', self.qos_profile)
        
        # topic: /fmu/in/vehicle_trajectory_bezier
        # type: px4_msgs::msg::VehicleTrajectoryBezier
        self.px4_fmu_in_vehicle_trajectory_bezier_publisher = self.create_publisher(VehicleTrajectoryBezier, '/fmu/in/vehicle_trajectory_bezier', self.qos_profile)
        
        # topic: /fmu/in/vehicle_trajectory_waypoint
        # type: px4_msgs::msg::VehicleTrajectoryWaypoint
        self.px4_fmu_in_vehicle_trajectory_waypoint_publisher = self.create_publisher(VehicleTrajectoryWaypoint, '/fmu/in/vehicle_trajectory_waypoint', self.qos_profile)
        
        # topic: /fmu/in/vehicle_visual_odometry
        # type: px4_msgs::msg::VehicleOdometry
        self.px4_fmu_in_vehicle_visual_odometry_publisher = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.qos_profile)



    # 所有发布者的控制函数 ----------------------------------------------------------------------------------------------------------------------------------
    '''
    #topic: /fmu/in/actuator_motors
    #type:  px4_msgs::msg::ActuatorMotors
    self.px4_fmu_in_actuator_motors_publisher = self.create_publisher(ActuatorMotors, '/fmu/in/actuator_motors', self.qos_profile)
    def publish_px4_fmu_in_actuator_motors(self):
        msg = ActuatorMotors()
        msg.actuator_motors = [0.0] * 8  # Assuming 8 motors
        self.px4_fmu_in_actuator_motors_publisher.publish(msg)

    # topic: /fmu/in/actuator_servos
    # type:  px4_msgs::msg::ActuatorServos
    self.px4_fmu_in_actuator_servos_publisher = self.create_publisher(ActuatorServos, '/fmu/in/actuator_servos', self.qos_profile)

    # topic: /fmu/in/arming_check_reply
    # type:  px4_msgs::msg::ArmingCheckReply
    self.px4_fmu_in_arming_check_reply_publisher = self.create_publisher(ArmingCheckReply, '/fmu/in/arming_check_reply', self.qos_profile)

    # topic: /fmu/in/aux_global_position
    # type:  px4_msgs::msg::VehicleGlobalPosition
    self.px4_fmu_in_aux_global_position_publisher = self.create_publisher(VehicleGlobalPosition, '/fmu/in/aux_global_position', self.qos_profile)

    # topic: /fmu/in/config_control_setpoints
    # type:  px4_msgs::msg::VehicleControlMode
    self.px4_fmu_in_config_control_setpoints_publisher = self.create_publisher(VehicleControlMode, '/fmu/in/config_control_setpoints', self.qos_profile)

    # topic: /fmu/in/config_overrides_request
    # type:  px4_msgs::msg::ConfigOverrides
    self.px4_fmu_in_config_overrides_request_publisher = self.create_publisher(ConfigOverrides, '/fmu/in/config_overrides_request', self.qos_profile)

    # topic: /fmu/in/differential_drive_setpoint
    # type: px4_msgs::msg::DifferentialDriveSetpoint
    self.px4_fmu_in_differential_drive_setpoint_publisher = self.create_publisher(DifferentialDriveSetpoint, '/fmu/in/differential_drive_setpoint', self.qos_profile)
    
    # topic: /fmu/in/goto_setpoint
    # type: px4_msgs::msg::GotoSetpoint
    self.px4_fmu_in_goto_setpoint_publisher = self.create_publisher(GotoSetpoint, '/fmu/in/goto_setpoint', self.qos_profile)
    
    # topic: /fmu/in/manual_control_input
    # type: px4_msgs::msg::ManualControlSetpoint
    self.px4_fmu_in_manual_control_input_publisher = self.create_publisher(ManualControlSetpoint, '/fmu/in/manual_control_input', self.qos_profile)
    
    # topic: /fmu/in/message_format_request
    # type: px4_msgs::msg::MessageFormatRequest
    self.px4_fmu_in_message_format_request_publisher = self.create_publisher(MessageFormatRequest, '/fmu/in/message_format_request', self.qos_profile)
    
    # topic: /fmu/in/mode_completed
    # type: px4_msgs::msg::ModeCompleted
    self.px4_fmu_in_mode_completed_publisher = self.create_publisher(ModeCompleted, '/fmu/in/mode_completed', self.qos_profile)
    
    # topic: /fmu/in/obstacle_distance
    # type: px4_msgs::msg::ObstacleDistance
    self.px4_fmu_in_obstacle_distance_publisher = self.create_publisher(ObstacleDistance, '/fmu/in/obstacle_distance', self.qos_profile)
    
    # topic: /fmu/in/offboard_control_mode
    # type: px4_msgs::msg::OffboardControlMode
    self.px4_fmu_in_offboard_control_mode_publisher = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', self.qos_profile)
    
    # topic: /fmu/in/onboard_computer_status
    # type: px4_msgs::msg::OnboardComputerStatus
    self.px4_fmu_in_onboard_computer_status_publisher = self.create_publisher(OnboardComputerStatus, '/fmu/in/onboard_computer_status', self.qos_profile)
    
    # topic: /fmu/in/register_ext_component_request
    # type: px4_msgs::msg::RegisterExtComponentRequest
    self.px4_fmu_in_register_ext_component_request_publisher = self.create_publisher(RegisterExtComponentRequest, '/fmu/in/register_ext_component_request', self.qos_profile)
    
    # topic: /fmu/in/sensor_optical_flow
    # type: px4_msgs::msg::SensorOpticalFlow
    self.px4_fmu_in_sensor_optical_flow_publisher = self.create_publisher(SensorOpticalFlow, '/fmu/in/sensor_optical_flow', self.qos_profile)
    
    # topic: /fmu/in/telemetry_status
    # type: px4_msgs::msg::TelemetryStatus
    self.px4_fmu_in_telemetry_status_publisher = self.create_publisher(TelemetryStatus, '/fmu/in/telemetry_status', self.qos_profile)
    
    # topic: /fmu/in/trajectory_setpoint
    # type: px4_msgs::msg::TrajectorySetpoint
    self.px4_fmu_in_trajectory_setpoint_publisher = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', self.qos_profile)
    
    # topic: /fmu/in/unregister_ext_component
    # type: px4_msgs::msg::UnregisterExtComponent
    self.px4_fmu_in_unregister_ext_component_publisher = self.create_publisher(UnregisterExtComponent, '/fmu/in/unregister_ext_component', self.qos_profile)
    
    # topic: /fmu/in/vehicle_attitude_setpoint
    # type: px4_msgs::msg::VehicleAttitudeSetpoint
    self.px4_fmu_in_vehicle_attitude_setpoint_Pubvehicle_attitude_setpointlisher_publisher = self.create_publisher(VehicleAttitudeSetpoint, '/fmu/in/vehicle_attitude_setpoint', self.qos_profile)
    
    # topic: /fmu/in/vehicle_command
    # type: px4_msgs::msg::VehicleCommand
    self.px4_fmu_in_vehicle_command_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', self.qos_profile)
    
    # topic: /fmu/in/vehicle_command_mode_executor
    # type: px4_msgs::msg::VehicleCommand
    self.px4_fmu_in_vehicle_command_mode_executor_publisher = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command_mode_executor', self.qos_profile)
    
    # topic: /fmu/in/vehicle_mocap_odometry
    # type: px4_msgs::msg::VehicleOdometry
    self.px4_fmu_in_vehicle_mocap_odometry_publisher = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_mocap_odometry', self.qos_profile)
    
    # topic: /fmu/in/vehicle_rates_setpoint
    # type: px4_msgs::msg::VehicleRatesSetpoint
    self.px4_fmu_in_vehicle_rates_setpoint_publisher = self.create_publisher(VehicleRatesSetpoint, '/fmu/in/vehicle_rates_setpoint', self.qos_profile)
    
    # topic: /fmu/in/vehicle_thrust_setpoint
    # type: px4_msgs::msg::VehicleThrustSetpoint
    self.px4_fmu_in_vehicle_thrust_setpoint_publisher = self.create_publisher(VehicleThrustSetpoint, '/fmu/in/vehicle_thrust_setpoint', self.qos_profile)
    
    # topic: /fmu/in/vehicle_torque_setpoint
    # type: px4_msgs::msg::VehicleTorqueSetpoint
    self.px4_fmu_in_vehicle_torque_setpoint_publisher = self.create_publisher(VehicleTorqueSetpoint, '/fmu/in/vehicle_torque_setpoint', self.qos_profile)
    
    # topic: /fmu/in/vehicle_trajectory_bezier
    # type: px4_msgs::msg::VehicleTrajectoryBezier
    self.px4_fmu_in_vehicle_trajectory_bezier_publisher = self.create_publisher(VehicleTrajectoryBezier, '/fmu/in/vehicle_trajectory_bezier', self.qos_profile)
    
    # topic: /fmu/in/vehicle_trajectory_waypoint
    # type: px4_msgs::msg::VehicleTrajectoryWaypoint
    self.px4_fmu_in_vehicle_trajectory_waypoint_publisher = self.create_publisher(VehicleTrajectoryWaypoint, '/fmu/in/vehicle_trajectory_waypoint', self.qos_profile)
    
    # topic: /fmu/in/vehicle_visual_odometry
    # type: px4_msgs::msg::VehicleOdometry
    self.px4_fmu_in_vehicle_visual_odometry_publisher = self.create_publisher(VehicleOdometry, '/fmu/in/vehicle_visual_odometry', self.qos_profile)
    '''




    #  创建所有的PX4相关话题的订阅者 ---------------------------------------------------------------------------------------------------------------------------------
    def create_all_px4_subscribers(self):
        #topic: /fmu/out/register_ext_component_reply
        #type: px4_msgs::msg::RegisterExtComponentReply
        self.px4_fmu_out_register_ext_component_reply_subscriber = self.create_subscription(
            RegisterExtComponentReply, '/fmu/out/register_ext_component_reply', self.register_ext_component_reply_callback, self.qos_profile)
        self.px4_fmu_out_register_ext_component_reply = RegisterExtComponentReply()

        #topic: /fmu/out/airspeed_validated
        #type: px4_msgs::msg::AirspeedValidated
        self.px4_fmu_out_airspeed_validated_subscriber = self.create_subscription(
            AirspeedValidated, '/fmu/out/airspeed_validated', self.airspeed_validated_callback, self.qos_profile)
        self.px4_fmu_out_airspeed_validated = AirspeedValidated()

        #topic: /fmu/out/arming_check_request
        #type: px4_msgs::msg::ArmingCheckRequest
        self.px4_fmu_out_arming_check_request_subscriber = self.create_subscription(
            ArmingCheckRequest, '/fmu/out/arming_check_request', self.arming_check_request_callback, self.qos_profile)
        self.px4_fmu_out_arming_check_request = ArmingCheckRequest()
        
        #topic: /fmu/out/mode_completed
        #type: px4_msgs::msg::ModeCompleted
        self.px4_fmu_out_mode_completed_subscriber = self.create_subscription(
            ModeCompleted, '/fmu/out/mode_completed', self.mode_completed_callback, self.qos_profile)
        self.px4_fmu_out_mode_completed = ModeCompleted()
        
        #topic: /fmu/out/battery_status
        #type: px4_msgs::msg::BatteryStatus
        self.px4_fmu_out_battery_status_subscriber = self.create_subscription(
            BatteryStatus, '/fmu/out/battery_status', self.battery_status_callback, self.qos_profile)
        self.px4_fmu_out_battery_status = BatteryStatus()
        
        #topic: /fmu/out/collision_constraints	
        #type: px4_msgs::msg::CollisionConstraints
        self.px4_fmu_out_collision_constraints_subscriber = self.create_subscription(
            CollisionConstraints, '/fmu/out/collision_constraints', self.collision_constraints_callback, self.qos_profile)
        self.px4_fmu_out_collision_constraints = CollisionConstraints()
        
        #topic: /fmu/out/estimator_status_flags
        #type: px4_msgs::msg::EstimatorStatusFlags
        self.px4_fmu_out_estimator_status_flags_subscriber = self.create_subscription(
            EstimatorStatusFlags, '/fmu/out/estimator_status_flags', self.estimator_status_flags_callback, self.qos_profile)
        self.px4_fmu_out_estimator_status_flags = EstimatorStatusFlags()
        
        #topic: /fmu/out/failsafe_flags
        #type: px4_msgs::msg::FailsafeFlags
        self.px4_fmu_out_failsafe_flags_subscriber = self.create_subscription(
            FailsafeFlags, '/fmu/out/failsafe_flags', self.failsafe_flags_callback, self.qos_profile)
        self.px4_fmu_out_failsafe_flags = FailsafeFlags()
        
        #topic: /fmu/out/manual_control_setpoint
        #type: px4_msgs::msg::ManualControlSetpoint
        self.px4_fmu_out_manual_control_setpoint_subscriber = self.create_subscription(
            ManualControlSetpoint, '/fmu/out/manual_control_setpoint', self.manual_control_setpoint_callback, self.qos_profile)
        self.px4_fmu_out_manual_control_setpoint = ManualControlSetpoint()
        
        #topic: /fmu/out/message_format_response
        #type: px4_msgs::msg::MessageFormatResponse
        self.px4_fmu_out_message_format_response_subscriber = self.create_subscription(
            MessageFormatResponse, '/fmu/out/message_format_response', self.message_format_response_callback, self.qos_profile)
        self.px4_fmu_out_message_format_response = MessageFormatResponse()
        
        #topic: /fmu/out/mission
        #type: px4_msgs::msg::Mission
        self.px4_fmu_out_mission_subscriber = self.create_subscription(
            Mission, '/fmu/out/mission', self.mission_callback, self.qos_profile)
        self.px4_fmu_out_mission = Mission()
        
        #topic: /fmu/out/mission_result
        #type: px4_msgs::msg::MissionResult
        self.px4_fmu_out_mission_result_subscriber = self.create_subscription(
            MissionResult, '/fmu/out/mission_result', self.mission_result_callback, self.qos_profile)
        self.px4_fmu_out_mission_result = MissionResult()
        
        #topic: /fmu/out/position_setpoint_triplet
        #type: px4_msgs::msg::PositionSetpointTriplet
        self.px4_fmu_out_position_setpoint_triplet_subscriber = self.create_subscription(
            PositionSetpointTriplet, '/fmu/out/position_setpoint_triplet', self.position_setpoint_triplet_callback, self.qos_profile)
        self.px4_fmu_out_position_setpoint_triplet = PositionSetpointTriplet()
        
        #topic: /fmu/out/sensor_combined
        #type: px4_msgs::msg::SensorCombined
        self.px4_fmu_out_sensor_combined_subscriber = self.create_subscription(
            SensorCombined, '/fmu/out/sensor_combined', self.sensor_combined_callback, self.qos_profile)
        self.px4_fmu_out_sensor_combined = SensorCombined()
        
        #topic: /fmu/out/timesync_status
        #type: px4_msgs::msg::TimesyncStatus
        self.px4_fmu_out_timesync_status_subscriber = self.create_subscription(
            TimesyncStatus, '/fmu/out/timesync_status', self.timesync_status_callback, self.qos_profile)
        self.px4_fmu_out_timesync_status = TimesyncStatus()
        
        #topic: /fmu/out/vehicle_attitude
        #type: px4_msgs::msg::VehicleAttitude
        self.px4_fmu_out_vehicle_attitude_subscriber = self.create_subscription(
            VehicleAttitude, '/fmu/out/vehicle_attitude', self.vehicle_attitude_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_attitude = VehicleAttitude()
        
        #topic: /fmu/out/vehicle_control_mode
        #type: px4_msgs::msg::VehicleControlMode
        self.px4_fmu_out_vehicle_control_mode_subscriber = self.create_subscription(
            VehicleControlMode, '/fmu/out/vehicle_control_mode', self.vehicle_control_mode_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_control_mode = VehicleControlMode()
        
        #topic: /fmu/out/vehicle_command_ack
        #type: px4_msgs::msg::VehicleCommandAck
        self.px4_fmu_out_vehicle_command_ack_subscriber = self.create_subscription(
            VehicleCommandAck, '/fmu/out/vehicle_command_ack', self.vehicle_command_ack_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_command_ack = VehicleCommandAck()
        
        #topic: /fmu/out/vehicle_global_position
        #type: px4_msgs::msg::VehicleGlobalPosition
        self.px4_fmu_out_vehicle_global_position_subscriber = self.create_subscription(
            VehicleGlobalPosition, '/fmu/out/vehicle_global_position', self.vehicle_global_position_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_global_position = VehicleGlobalPosition()
        
        #topic: /fmu/out/vehicle_gps_position
        #type: px4_msgs::msg::SensorGps
        self.px4_fmu_out_vehicle_gps_position_subscriber = self.create_subscription(
            SensorGps, '/fmu/out/vehicle_gps_position', self.vehicle_gps_position_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_gps_position = SensorGps()
        
        #topic: /fmu/out/vehicle_local_position
        #type: px4_msgs::msg::VehicleLocalPosition
        self.px4_fmu_out_vehicle_local_position_subscriber = self.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.vehicle_local_position_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_local_position = VehicleLocalPosition()
        
        #topic: /fmu/out/vehicle_odometry
        #type: px4_msgs::msg::VehicleOdometry
        self.px4_fmu_out_vehicle_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.vehicle_odometry_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_odometry = VehicleOdometry()
        
        #topic: /fmu/out/vehicle_status
        #type: px4_msgs::msg::VehicleStatus
        self.px4_fmu_out_vehicle_status_subscriber = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status', self.vehicle_status_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_status = VehicleStatus()
        
        #topic: /fmu/out/vehicle_trajectory_waypoint_desired
        #type: px4_msgs::msg::VehicleTrajectoryWaypoint
        self.px4_fmu_out_vehicle_trajectory_waypoint_desired_subscriber = self.create_subscription(
            VehicleTrajectoryWaypoint, '/fmu/out/vehicle_trajectory_waypoint_desired', self.vehicle_trajectory_waypoint_desired_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_trajectory_waypoint_desired = VehicleTrajectoryWaypoint()
        
        #topic: /fmu/out/vehicle_visual_odometry
        #type: px4_msgs::msg::VehicleOdometry
        self.px4_fmu_out_vehicle_visual_odometry_subscriber = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_visual_odometry', self.vehicle_visual_odometry_callback, self.qos_profile)
        self.px4_fmu_out_vehicle_visual_odometry = VehicleOdometry()


    # PX所有订阅话题的回调函数 ----------------------------------------------------------------------------------------------------------------------------------
    def register_ext_component_reply_callback(self, register_ext_component_reply):
        self.px4_fmu_out_register_ext_component_reply = register_ext_component_reply
        #self.get_logger().info(f"Received register_ext_component_reply: {register_ext_component_reply}")

    def airspeed_validated_callback(self, airspeed_validated):
        self.px4_fmu_out_airspeed_validated = airspeed_validated
        #self.get_logger().info(f"Received airspeed_validated: {airspeed_validated}")

    def arming_check_request_callback(self, arming_check_request):
        self.px4_fmu_out_arming_check_request = arming_check_request
        #self.get_logger().info(f"Received arming_check_request: {arming_check_request}")

    def mode_completed_callback(self, mode_completed):
        self.px4_fmu_out_mode_completed = mode_completed
        #self.get_logger().info(f"Received mode_completed: {mode_completed}")

    def battery_status_callback(self, battery_status):
        self.px4_fmu_out_battery_status = battery_status
        #self.get_logger().info(f"Received battery_status: {battery_status}")

    def collision_constraints_callback(self, collision_constraints):
        self.px4_fmu_out_collision_constraints = collision_constraints
        #self.get_logger().info(f"Received collision_constraints: {collision_constraints}")

    def estimator_status_flags_callback(self, estimator_status_flags):
        self.px4_fmu_out_estimator_status_flags = estimator_status_flags
        #self.get_logger().info(f"Received estimator_status_flags: {estimator_status_flags}")

    def failsafe_flags_callback(self, failsafe_flags):
        self.px4_fmu_out_failsafe_flags = failsafe_flags
        #self.get_logger().info(f"Received failsafe_flags: {failsafe_flags}")

    def manual_control_setpoint_callback(self, manual_control_setpoint):
        self.px4_fmu_out_manual_control_setpoint = manual_control_setpoint
        #self.get_logger().info(f"Received manual_control_setpoint: {manual_control_setpoint}")

    def message_format_response_callback(self, message_format_response):
        self.px4_fmu_out_message_format_response = message_format_response
        #self.get_logger().info(f"Received message_format_response: {message_format_response}")

    def mission_callback(self, mission):
        self.px4_fmu_out_mission = mission
        #self.get_logger().info(f"Received mission: {mission}")

    def mission_result_callback(self, mission_result):
        self.px4_fmu_out_mission_result = mission_result
        #self.get_logger().info(f"Received mission_result: {mission_result}")

    def position_setpoint_triplet_callback(self, position_setpoint_triplet):
        self.px4_fmu_out_position_setpoint_triplet = position_setpoint_triplet
        #self.get_logger().info(f"Received position_setpoint_triplet: {position_setpoint_triplet}")

    def sensor_combined_callback(self, sensor_combined):
        self.px4_fmu_out_sensor_combined = sensor_combined
        #self.get_logger().info(f"Received sensor_combined: {sensor_combined}")

    def timesync_status_callback(self, timesync_status):
        self.px4_fmu_out_timesync_status = timesync_status
        #self.get_logger().info(f"Received timesync_status: {timesync_status}")

    def vehicle_attitude_callback(self, vehicle_attitude):
        self.px4_fmu_out_vehicle_attitude = vehicle_attitude
        #self.get_logger().info(f"Received vehicle_attitude: {vehicle_attitude}")

    def vehicle_control_mode_callback(self, vehicle_control_mode):
        self.px4_fmu_out_vehicle_control_mode = vehicle_control_mode
        #self.get_logger().info(f"Received vehicle_control_mode: {vehicle_control_mode}")

    def vehicle_command_ack_callback(self, vehicle_command_ack):
        self.px4_fmu_out_vehicle_command_ack = vehicle_command_ack
        #self.get_logger().info(f"Received vehicle_command_ack: {vehicle_command_ack}")

    def vehicle_global_position_callback(self, vehicle_global_position):
        self.px4_fmu_out_vehicle_global_position = vehicle_global_position
        #self.get_logger().info(f"Received vehicle_global_position: {vehicle_global_position}")

    def vehicle_gps_position_callback(self, vehicle_gps_position):
        self.px4_fmu_out_vehicle_gps_position = vehicle_gps_position
        #self.get_logger().info(f"Received vehicle_gps_position: {vehicle_gps_position}")

    def vehicle_local_position_callback(self, vehicle_local_position):
        self.px4_fmu_out_vehicle_local_position = vehicle_local_position
        #self.get_logger().info(f"Received vehicle_local_position: {vehicle_local_position}")

    def vehicle_odometry_callback(self, vehicle_odometry):
        self.px4_fmu_out_vehicle_odometry = vehicle_odometry
        #self.get_logger().info(f"Received vehicle_odometry: {vehicle_odometry}")

    def vehicle_status_callback(self, vehicle_status):
        self.px4_fmu_out_vehicle_status = vehicle_status
        self.get_logger().info(f"Received vehicle_status: {vehicle_status}")

    def vehicle_trajectory_waypoint_desired_callback(self, vehicle_trajectory_waypoint_desired):
        self.px4_fmu_out_vehicle_trajectory_waypoint_desired = vehicle_trajectory_waypoint_desired
        #self.get_logger().info(f"Received vehicle_trajectory_waypoint_desired: {vehicle_trajectory_waypoint_desired}")

    def vehicle_visual_odometry_callback(self, vehicle_visual_odometry):
        self.px4_fmu_out_vehicle_visual_odometry = vehicle_visual_odometry
        #self.get_logger().info(f"Received vehicle_visual_odometry: {vehicle_visual_odometry}")
    

    def timer_print_subscribe_callback(self) -> None:
        time.sleep(1.5)
    
    self.get_logger().info(f"Received vehicle_status: {vehicle_status.arming_state}")

def main(args=None) -> None:
    print('px4 test node...')
    rclpy.init(args=args)
    px4_test = px4_test_def()
    rclpy.spin(px4_test)
    px4_test.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)
