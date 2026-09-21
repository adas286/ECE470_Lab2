#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from ur_msgs.srv import SetIO
from ur_msgs.msg import IOStates
import time
import numpy as np
from math import pi
import sys
class JointAngles:
    def __init__(self):
        self.name = ["", "", "", "", "", ""]  #could have also done [""] * 6
        self.position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]

# UR3e home position
home = np.radians([120, -90, 90, -90, -90, 0])

# Hanoi tower location 
Q11 = [191.56*pi/180.0, -27.82*pi/180.0, 56.14*pi/180.0, -122.90*pi/180.0, -90.57*pi/180.0, 0.11*pi/180.0]
Q21 = [191.56*pi/180.0, -31.67*pi/180.0, 54.99*pi/180.0, -117.90*pi/180.0, -90.55*pi/180.0, 0.11*pi/180.0]
Q31 = [191.56*pi/180.0, -35.00*pi/180.0, 52.31*pi/180.0, -111.88*pi/180.0, -90.53*pi/180.0, 0.10*pi/180.0]
Q12 = [176.98*pi/180.0, -48.90*pi/180.0, 103.63*pi/180.0, -149.31*pi/180.0, -89.48*pi/180.0, (343.51-360)*pi/180.0]
Q22 = [176.99*pi/180.0, -55.41*pi/180.0, 102.58*pi/180.0, -141.75*pi/180.0, -89.46*pi/180.0, (345.51-360)*pi/180.0]
Q32 = [176.99*pi/180.0, -60.82*pi/180.0, 100.34*pi/180.0, -134.11*pi/180.0, -89.46*pi/180.0, (345.11-360)*pi/180.0]
Q13 = [217.19*pi/180.0, -51.48*pi/180.0, 107.6*pi/180.0, -149.97*pi/180.0, -92.57*pi/180.0, 25.60*pi/180.0]
Q23 = [217.42*pi/180.0, -57.37*pi/180.0, 105.24*pi/180.0, -141.72*pi/180.0, -92.56*pi/180.0, 25.83*pi/180.0]
Q33 = [217.42*pi/180.0, -62.74*pi/180.0, 102.97*pi/180.0, -134.08*pi/180.0, -92.55*pi/180.0, 25.83*pi/180.0]

############## Your Code Start Here ##############
"""
TODO: Initialize Q matrix
"""

Q = [ [Q11, Q12, Q13], \
      [Q21, Q22, Q23], \
      [Q31, Q32, Q33] ]
############### Your Code End Here ###############
class UR3e(Node):
    def __init__(self):
        super().__init__('ur3e')

        # Publishers
        self.trajectory_pub = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)

        # Subscribers
        self.joint_state_sub = self.create_subscription(JointState, '/joint_states', self.joint_state_callback, 10)

        ############## Your Code Start Here ##############
        # TODO: define a ROS subscriber for gripper input message and corresponding callback function
        # ROS2 gripper input topic: /io_and_status_controller/io_states
        self.gripper_input = self.create_subscription(IOStates, '/io_and_status_controller/io_states', self.io_state_callback, 10)


        ############### Your Code End Here ###############

        # Service clients
        self.io_client = self.create_client(SetIO, '/io_and_status_controller/set_io')
        while not self.io_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('IO service not available, waiting...')

        # State variables
        self.current_joint_state = None
        self.analog_in_0_value = 0
        self.current_JointAngles = JointAngles()
        self.joint_names = [
            'shoulder_pan_joint', 'shoulder_lift_joint', 'elbow_joint',
            'wrist_1_joint', 'wrist_2_joint', 'wrist_3_joint'
        ] # shoulder_pan_joint is the base rotation joint

    def joint_state_callback(self, msg):
        self.current_joint_state = msg  # Currently only used to check if messages have arrived
        index_inOrder = 0
        for name in self.joint_names:
            index_outofOrder = msg.name.index(name)
            self.current_JointAngles.name[index_inOrder] = name
            self.current_JointAngles.position[index_inOrder] = msg.position[index_outofOrder]
            index_inOrder = index_inOrder + 1 


    def io_state_callback(self, msg):
    ############## Your Code Start Here ##############
        """
        TODO: define a ROS topic callback funtion that 
        receives and stores the state of  the suction cup
        Whenever /io_and_status_controller/io_states 
        publishes this info, this callback function is
        called.
        """
        self.analog_in_0_value = msg.analog_in_states[0].state  
        #print(f"Analog input 0 value: {self.analog_in_0_value}")  # Optional: print the value for debugging



    ############### Your Code End Here ###############

    def set_io(self, pin, state):
        req = SetIO.Request()
        req.fun = 1
        req.pin = pin
        req.state = state
        future = self.io_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()


    def move_arm(self, target):
        if self.current_joint_state is None:
            self.get_logger().error("No joint state received!")
            return False

        V_MAX = 1#2.09    # rad/s
        A_MAX = 0.8#2.79   # rad/s^2
        MIN_DURATION = 1
        MAX_DURATION = 8.0

        deltas = []
        for i in range(6):
            deltas.append(abs(self.current_JointAngles.position[i] - target[i]))


        max_delta = max(deltas)
        t_acc = V_MAX / A_MAX
        d_acc = 0.5 * A_MAX * (t_acc ** 2)
        if max_delta > 2 * d_acc:
            # trapezoidal velocity profile
            t_total = 2 * t_acc + (max_delta - 2 * d_acc) / V_MAX
        else:
            # triangular velocity profile
            t_total = 2 * (max_delta / A_MAX) ** 0.5

        duration = max(MIN_DURATION, min(t_total, MAX_DURATION))

        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = self.joint_names

        # Start immediately when the controller receives it
        trajectory_msg.header.stamp.sec = 0
        trajectory_msg.header.stamp.nanosec = 0

        # Anchor point: current measured joint state at t = 0
        p0 = JointTrajectoryPoint()
        p0.positions = self.current_JointAngles.position
        p0.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # starting at rest
        p0.time_from_start.sec = 0
        p0.time_from_start.nanosec = 0
        trajectory_msg.points.append(p0)

        # Goal point
        p1 = JointTrajectoryPoint()
        p1.positions = target
        p1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] # end at rest, 2 point trajectory
        p1.accelerations = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0] #end at rest.
        p1.time_from_start.sec = int(duration)
        p1.time_from_start.nanosec = int((duration - int(duration)) * 1e9)
        trajectory_msg.points.append(p1)

        self.trajectory_pub.publish(trajectory_msg)

        self.get_logger().info(f'Moving to position: {np.degrees(target)}')

        # Wait for movement completion
        start_time = time.time()
        while time.time() - start_time < duration + 2:
            rclpy.spin_once(self, timeout_sec=0.1)

            deltas = []
            for i in range(6):
                deltas.append(abs(self.current_JointAngles.position[i] - target[i]))
            if all(delta < 0.001 for delta in deltas):
                time.sleep(0.25)
                return True
        return False



    def move_block(self, start_tower, start_height, end_tower, end_height):
        global Q
    ############## Your Code Start Here ##############
    # TODO: add code to move block from start tower and height to end tower and height
    ### Hint: Use the Q array to map out your towers by location and "height".
        if not self.move_arm(Q[start_height][start_tower]):
            self.get_logger().error(f"Failed to reach pickup pose (tower {start_tower}, height {start_height})")
            return 1
 
        self.set_io(0, 1.0) 
        time.sleep(1.0) 

        self.move_arm(home)  # Move up to pick up the block
        if self.analog_in_0_value <= 2.0:  # Check if the block is detected
            self.set_io(0, 0.0)
            self.get_logger().error(
                f"No block detected at tower {start_tower}, height {start_height}")
            sys.exit()
 
        # Go to the place pose
        if not self.move_arm(Q[end_height][end_tower]):
            self.get_logger().error(
                f"Failed to reach place pose (tower {end_tower}, height {end_height})")
            sys.exit()
 
        self.set_io(0, 0.0) 
        time.sleep(0.5)
        self.move_arm(home)  # Move up after placing the block
 
        return 0


    ############### Your Code End Here ###############
    def solve_hanoi(self, n, source, target, heights):
        ##Move the top n blocks from `source` tower to `target` tower.`heights` is a length-3 list tracking how many blocks currently sit on each tower; it is updated in place as moves happen. The auxiliary tower is inferred automatically since 0 + 1 + 2 == 3.
        if n == 0:
            return True
 
        aux = 3 - source - target
 
        if not self.solve_hanoi(n - 1, source, aux, heights):
            return False
 
        start_height = heights[source] - 1
        end_height = heights[target]
 
        if self.move_block(source, start_height, target, end_height) != 0:
            return False
    
        heights[source] -= 1
        heights[target] += 1
 
        return self.solve_hanoi(n - 1, aux, target, heights)


def main(args=None):
    input("Check if the UR3e is in 'Remote' Mode?\n\
    Check if the UR3e is initialized and in 'Normal' state.\n\
    Have you run the ROS2 launch statement?\n\
    If there was an UR3e emergency stop or error, Ctrl-C the ros2 launch and rerun.\n\
    \n\
    Press <Enter> to Continue.")
    rclpy.init(args=args)
    node = UR3e()
    executor = SingleThreadedExecutor()
    executor.add_node(node)

    while node.current_joint_state is None:
        executor.spin_once(timeout_sec=0.1)
        node.get_logger().info("Waiting for joint state messages...")
        time.sleep(0.5)

    ############## Your Code Start Here ##############
    # TODO: modify the code below so that program can get user input
    try:
        start_tower = int(input("Enter start tower (0, 1, or 2): "))
        end_tower = int(input("Enter end tower (0, 1, or 2): "))
 
        if not (0 <= start_tower <= 2) or not (0 <= end_tower <= 2) or start_tower == end_tower:
            print("Invalid tower choice.")
            sys.exit()
 
        node.move_arm(home)
 
        # All 3 blocks start on start_tower; the other two towers are empty
        heights = [0, 0, 0]
        heights[start_tower] = 3
 
        try:
            success = node.solve_hanoi(3, start_tower, end_tower, heights)
        except BlockNotDetectedError as e:
            node.get_logger().error(f"Halting: {e}")
            success = False
 
        if success:
            node.get_logger().info("Tower of Hanoi solved successfully.")
        else:
            node.get_logger().error("Hanoi solve aborted due to a failed move.")
 
        node.move_arm(home)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
