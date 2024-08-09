
import math, rclpy, time, operator, can
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist
from robot_interfaces.msg import Status, CanMotorData
from robot_interfaces.srv import SendCanCommand
from myactuator_lib import Motor as MyActuatorMotor
from rclpy.node import Node


class DriveBaseControlNode(Node):

    def __init__(self):

        super().__init__('drivebase_control_node')

        self.declare_parameter('debugging', True)

        # CAN Parameters
        self.declare_parameter('can_srv_name','send_n_recv_can_msg')
        self.declare_parameter('can_topic_name', 'outgoing_can_commands') 
        self.declare_parameter('can_motor_data_topic','can_motor_speed_response_data')


        self.declare_parameter('front_left_motor_arbitration_id', 0x141)  #ID 1
        self.declare_parameter('back_left_motor_arbitration_id', 0x142)   #ID 2
        self.declare_parameter('front_right_motor_arbitration_id', 0x143) #ID 3
        self.declare_parameter('back_right_motor_arbitration_id', 0x144)  #ID 4

        # Motor Parameters
        self.declare_parameter('gear_reduction_ratio',1/35)
        self.declare_parameter('track_width', 0.914)
        self.declare_parameter('wheel_radius', 0.254)

        # Command Velocity Parameters
        self.declare_parameter('cmd_vel_topic', 'cmd_vel')
        self.declare_parameter('max_linear_speed_mps',10.0)
        
        # Joy Parameters
        self.declare_parameter('joy_topic', 'joy')
        self.declare_parameter('enable_button', 2)          # Xbox X button
        self.declare_parameter('enable_turbo_button', 5)    # Xbox Right Trigger

        # Robot Parameters
        self.declare_parameter('isHeimdall', True)

        # Heartbeat Parameters
        self.declare_parameter('doHeartbeat', False)

        # Create the CAN service client 
        self.can_service_client = self.create_client(
            srv_type=SendCanCommand,
            srv_name= self.get_parameter('can_srv_name').get_parameter_value().string_value,
            )
        
        # Can Data Public
        self.motor_data_pub = self.create_publisher(
            msg_type= CanMotorData,
            topic = self.get_parameter('can_motor_data_topic').get_parameter_value().string_value,
            qos_profile=10
        )

        # Command Velocity Subscription
        self.cmd_vel_sub = self.create_subscription(
            msg_type= Twist,
            topic= self.get_parameter('cmd_vel_topic').get_parameter_value().string_value,
            callback= self.send_drivebase_command,
            qos_profile= 1
            )
        
        # Joystick subscription
        self.joy_sub = self.create_subscription(
            msg_type= Joy,
            topic= self.get_parameter('joy_topic').get_parameter_value().string_value,
            callback= self.zero_command_wrapper,
            qos_profile= 1
            )
        
        # Heartbeat subscription
        self.heartbead_sub = self.create_subscription(
            msg_type= Status,
            topic='reciever_communication',
            callback= self.heartbeat_callback,
            qos_profile=10
        )

        # Set up the motors
        self.front_left_motor = MyActuatorMotor(
            self.get_parameter('front_left_motor_arbitration_id').get_parameter_value().integer_value
            )
        self.back_left_motor = MyActuatorMotor(
            self.get_parameter('back_left_motor_arbitration_id').get_parameter_value().integer_value
            )
        self.front_right_motor = MyActuatorMotor(
            self.get_parameter('front_right_motor_arbitration_id').get_parameter_value().integer_value
            )
        self.back_right_motor = MyActuatorMotor(
            self.get_parameter('back_right_motor_arbitration_id').get_parameter_value().integer_value
            )       

        # Set up the heartbeat parameters
        if (not self.get_parameter('doHeartbeat').get_parameter_value().bool_value):
            self.cur_status = True
        else:
            self.cur_status = False
        self.offLock = False


    def heartbeat_callback(self, status_msg):
        if (not self.get_parameter('doHeartbeat').get_parameter_value().bool_value):
            self.cur_status = True
        else:
            self.cur_status = status_msg.isup

    # Check if the controller is actually doing things
    def zero_command_wrapper(self, joy_msg):

        # Get the values of the enable buttons
        enable = joy_msg.buttons[self.get_parameter('enable_button').get_parameter_value().integer_value]
        turbo  = joy_msg.buttons[self.get_parameter('enable_turbo_button').get_parameter_value().integer_value]

        # If neither enable buttons are pressed
        if (not enable and not turbo):
            self.offLock = True
            return
        self.offLock = False

    def send_drivebase_command(self, twist_msg):  

        # self.get_logger().info(f"x:{twist_msg.linear.x}\ty:{twist_msg.angular.y}")

        if (self.offLock or not self.cur_status):
            twist_msg = Twist()

        # Calculates max degrees per second for the motor
        def calc_dps(motor_side: str):
            
            track_width: float = (self.get_parameter('track_width').get_parameter_value().double_value)
            gear_ratio: float = self.get_parameter('gear_reduction_ratio').get_parameter_value().double_value
            wheel_radius: float = self.get_parameter('wheel_radius').get_parameter_value().double_value

            def calc_linear_vel(curr_motor_side: str):
                if curr_motor_side.upper() == "RIGHT":
                    op = operator.sub
                elif curr_motor_side.upper() == "LEFT":
                    op = operator.add
                else:
                    raise ValueError
                
                # If we're running on heimdall invert the output
                if (self.get_parameter('isHeimdall').get_parameter_value().bool_value):
                    return -1 * op(-1 * 2 * twist_msg.angular.z * track_width/2, 0.5 * twist_msg.linear.x)
                else:
                    return -1 * op(-1 * 2 * twist_msg.angular.z * track_width/2, 0.5 * twist_msg.linear.x)

            lin_vel = calc_linear_vel(motor_side)
            # self.get_logger().info(f"motor side: {motor_side} is {lin_vel}")


            max_dps = ((self.get_parameter('max_linear_speed_mps').get_parameter_value().double_value) / (2.0 * math.pi * wheel_radius)) * 360
            # self.get_logger().info(f"max dps: {max_dps}")

            def clamp(value, minimum, maximum):
                return max(minimum, min(value,maximum))
    
            speed = clamp(40 * (( lin_vel / (wheel_radius * 2 * math.pi)) * gear_ratio * 360),-200,200)
            # self.get_logger().info(f"speed: {speed}")
            return speed
        
        def send_can_message(can_command: can.Message):
            request = SendCanCommand.Request()
            request.arbitration_id = can_command.arbitration_id
            request.is_extended_id = can_command.is_extended_id
            request.byte_0 = can_command.data[0]
            request.byte_1 = can_command.data[1]
            request.byte_2 = can_command.data[2]
            request.byte_3 = can_command.data[3]
            request.byte_4 = can_command.data[4]
            request.byte_5 = can_command.data[5]
            request.byte_6 = can_command.data[6]
            request.byte_7 = can_command.data[7]
            
            self.future = self.can_service_client.call_async(request)
            rclpy.spin_until_future_complete(self, self.future)
            return self.future.result()
        
        def parse_speed_command_response(self, can_response: SendCanCommand.Response):

            response_data: CanMotorData = CanMotorData()
            response_data.timestamp = can_response.timestamp
            response_data.motor_id = can_response.recv_arbitration_id
            response_data.motor_temp_c = can_response.recv_byte_1
            response_data.torque_current_amps = ((can_response.recv_byte_3 << 8) | can_response.recv_byte_2 ) / 100
            response_data.motor_speed_dps = ((can_response.recv_byte_4 << 8) | can_response.recv_byte_5 )
            response_data.motor_angle_degrees = ((can_response.recv_byte_6 << 8) | can_response.recv_byte_7 )

            self.motor_data_pub.publish(response_data)


        left_dps = calc_dps('left')
        right_dps = calc_dps('right')

        self.get_logger().info(f"Left DPS: {left_dps}")
        self.get_logger().info(f"Right DPS: {right_dps}")

        # Send rpm values to the myactuator motors, Create motor command based on rpm value
        motor_commands = []
        for motor in [self.front_left_motor,self.back_left_motor]:
            motor_commands.append(motor.Speed_Closed_loop_Control_Command(left_dps))

        for motor in [self.front_right_motor,self.back_right_motor]:
            motor_commands.append(motor.Speed_Closed_loop_Control_Command(right_dps))

        # Send service request for a can command 
        for command in motor_commands:
            if self.get_parameter('debugging').get_parameter_value().bool_value:
                self.get_logger().info(str(command))

            self.get_logger().info('Sending command')
            can_response = send_can_message(command)
            self.get_logger().info('Finished sending command')

            parse_speed_command_response(can_response)

        # Motors spin (hopefully)


def main(args=None):

    rclpy.init(args=args)
    drivebase_node = DriveBaseControlNode()
    rclpy.spin(drivebase_node)
    drivebase_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
