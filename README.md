# drivebase_control_pkg

Message [Nate Adkins](mailto:npa00003@mix.wvu.edu) on Slack with any questions or suggestions

## Overview

This ROS 2 package provides two nodes for controlling a drivebase with motors, namely `drivebase_control_node` and `drivebase_control_node_w_srv`. The nodes subscribe to Twist messages (linear and angular velocities) and Joy messages (joystick inputs) to command the drivebase. They also provide options for sending CAN commands to control motors.

## Nodes

### Node: drivebase_control_node

This node subscribes to Twist and Joy messages to control the drivebase. It calculates the required motor speeds and sends CAN commands to the motors accordingly.

#### Parameters

- **debugging**: Enable or disable debugging output (default: False).
- **can_topic_name**: ROS topic for outgoing CAN commands (default: 'outgoing_can_commands').
- **front_left_motor_arbitration_id**: Arbitration ID for the front left motor (default: 0x141).
- **back_left_motor_arbitration_id**: Arbitration ID for the back left motor (default: 0x142).
- **front_right_motor_arbitration_id**: Arbitration ID for the front right motor (default: 0x143).
- **back_right_motor_arbitration_id**: Arbitration ID for the back right motor (default: 0x144).
- **gear_reduction_ratio**: Gear reduction ratio for motor control (default: 1/35).
- **track_width**: Width between the left and right wheels of the drivebase (default: 1.0).
- **wheel_radius**: Radius of the wheels on the drivebase (default: 1.0).
- **cmd_vel_topic**: ROS topic for Twist messages (linear and angular velocities) (default: 'cmd_vel').
- **max_linear_speed_mps**: Maximum linear speed in meters per second (default: 10.0).
- **joy_topic**: ROS topic for Joy messages (joystick inputs) (default: 'joy').
- **enable_button**: Joystick button to enable drivebase control (default: 2).
- **enable_turbo_button**: Joystick button for turbo mode (default: 5).
- **status_topic**: ROS topic for status messages (default: 'reciever_communication').
- **heartbeat_mode**: Enable or disable heartbeat mode for status checking (default: False).

#### Subscribed Topics

- **/cmd_vel**: Twist messages for controlling linear and angular velocities.
- **/joy**: Joy messages for joystick inputs.
- **/reciever_communication**: Status messages for communication status.

### Node: drivebase_control_node_w_srv

This node provides a service-based interface for controlling the drivebase. It uses a service client to send CAN commands to the motors.

#### Parameters

- **debugging**: Enable or disable debugging output (default: False).
- **can_srv_name**: Name of the ROS service for sending and receiving CAN commands (default: 'send_n_recv_can_msg').
- **front_left_motor_arbitration_id**: Arbitration ID for the front left motor (default: 0x141).
- **back_left_motor_arbitration_id**: Arbitration ID for the back left motor (default: 0x142).
- **front_right_motor_arbitration_id**: Arbitration ID for the front right motor (default: 0x143).
- **back_right_motor_arbitration_id**: Arbitration ID for the back right motor (default: 0x144).
- **gear_reduction_ratio**: Gear reduction ratio for motor control (default: 1/35).
- **track_width**: Width between the left and right wheels of the drivebase (default: 1.0).
- **wheel_radius**: Radius of the wheels on the drivebase (default: 1.0).
- **cmd_vel_topic**: ROS topic for Twist messages (linear and angular velocities) (default: 'cmd_vel').
- **max_linear_speed_mps**: Maximum linear speed in meters per second (default: 10.0).
- **joy_topic**: ROS topic for Joy messages (joystick inputs) (default: 'joy').
- **enable_button**: Joystick button to enable drivebase control (default: 2).
- **enable_turbo_button**: Joystick button for turbo mode (default: 5).
- **status_topic**: ROS topic for status messages (default: 'reciever_communication').
- **heartbeat_mode**: Enable or disable heartbeat mode for status checking (default: False).

#### Subscribed Topics

- **/cmd_vel**: Twist messages for controlling linear and angular velocities.
- **/joy**: Joy messages for joystick inputs.
- **/reciever_communication**: Status messages for communication status.

#### Services

- **/send_n_recv_can_msg**: Service for sending CAN commands and receiving the corresponding response.

## Building and Running

1. Clone this package into your ROS 2 workspace.

    ```bash
    cd <path_to_your_workspace>
    cd src
    git clone git@github.com:wvu-urc/drivebase_control_pkg.git
    ```

2. Build the ROS 2 workspace.

    ```bash
    cd ..
    colcon build
    ```

3. Source the ROS 2 workspace.

    ```bash
    source setup.bash
    ```

4. Run the `drivebase_control_node`.

    ```bash
    ros2 run drivebase_control_pkg drivebase_control_node
    ```
    
    ### **OR**
    
    Run the `drivebase_control_node_w_srv`.

    ```bash
    ros2 run drivebase_control_pkg drivebase_control_node_w_srv
    ```

6. Ensure that the required topics/services are being published/sent with appropriate message/service types.

## Additional Notes

- Make sure the CAN bus is properly connected and accessible at the specified channel.
- Check the udev rules for your CAN interface to ensure proper permissions.
- For debugging, you can set the `debugging` parameter to True in the launch files or directly from the command line.

## Known Limitations
The drivetrain control package works well, but does have some legacy code bits still included that were not used (like the comms heartbeat)

Some of the drive values are magic numbers, and the reason for the 'iswanderer2' parameter is to determine if it is our autonomy testbed or heimdall, which have different drive systems.

Only CAN is supported in this package, but small adjustments can be made (and have been made for other projects) to allow for UART control of the MyActuator Motors

Service Control was not tested or used on the robot, it should work but that is unknown

This should be used with the motor protection package as this package does not natively support command timeout and the robot can continue to travel if disconnected