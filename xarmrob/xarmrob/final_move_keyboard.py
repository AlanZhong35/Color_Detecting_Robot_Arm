#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
import traceback
from sensor_msgs.msg import JointState
from xarmrob_interfaces.srv import ME439XArmInverseKinematics


## Define a temporary function using Python "lambda" functionality to print colored text
coloredtext = lambda r, g, b, text: f'\033[38;2;{r};{g};{b}m{text}\033[38;2;255;255;255m'


class EndpointKeyboard(Node):
    def __init__(self):
        super().__init__('endpoint_keyboard')
       
        # Predefined locations dictionary
        self.predefined_locations = {
            'block': [0.25, 0, 0.05],
            'camera': [0.13, 0, 0.08],
            'bucket1': [0.02, -0.165, 0.11], # 'bucket1': [0, 0.14, 0.11],
            'bucket2': [0.02, 0.165, 0.11] #'bucket2': [0, -0.14, 0.11]
        }
       
        # Publisher for the Endpoint goal
        self.pub_joint_angles_desired = self.create_publisher(JointState,'/joint_angles_desired', 1)
       
        # Create the message, with a nominal pose
        # Default gripper state is open (0)
        self.ang_all = [0., -np.pi/2., np.pi/2., 0., 0., 0., 0.]
        self.gripper_state = 0  # 0 for open, pi/2 for closed
        self.joint_angles_desired_msg = JointState()
        self.joint_angles_desired_msg.name = ['base_joint', 'shoulder_joint', 'elbow_joint', 'forearm_joint', 'wrist_joint', 'fingers_joint', 'gripper']
        self.joint_angles_desired_msg.position = self.ang_all  # upright neutral position with open gripper


        # Client for the Inverse Dynamics service
        self.cli_inverse_kinematics = self.create_client(ME439XArmInverseKinematics, 'xarm_inverse_kinematics')
        while not self.cli_inverse_kinematics.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting...')
       
        # The Request message for the service
        self.request_inverse_kinematics = ME439XArmInverseKinematics.Request()
       
    def close_gripper(self):
        """Close the gripper (last link)"""
        # Set the last angle (gripper) to close position (np.pi/2)
        self.gripper_state = np.pi/2
        self.get_logger().info(coloredtext(50,255,50, 'Gripper Closed'))
        self.send_joint_angles_desired()

    def open_gripper(self):
        """Open the gripper (last link)"""
        # Set the last angle (gripper) to open position (0)
        self.gripper_state = 0
        self.get_logger().info(coloredtext(50,255,50, 'Gripper Opened'))
        self.send_joint_angles_desired()

    def endpoint_requests(self):
        # Run a loop that gets input from the user
        while True:
            try:
                in_string = input('Enter Endpoint Location:\n' +
                                '- As x,y,z coordinates (in meters)\n' +
                                '- Or predefined location (block, camera, bucket1, bucket2)\n' +
                                '- Type "g" to close gripper\n' +
                                '- Type "r" to open gripper\n' +
                                '- Ctrl-C and Enter to exit.\n').lower().strip()
            
                # Check for gripper commands
                if in_string == 'g':
                    self.close_gripper()
                    continue
                elif in_string == 'r':
                    self.open_gripper()
                    continue
            
                # Check if input is a predefined location
                if in_string in self.predefined_locations:
                    xyz_goal = self.predefined_locations[in_string]
                    prnttmpl = coloredtext(50,255,50,f'\n\tMoving to {in_string} at coordinates: [' + '{:.3f}, '*2 + '{:.3f}]')
                    self.get_logger().info(prnttmpl.format(*xyz_goal))
                else:
                    # Try to parse custom coordinates
                    try:
                        in_floats = list(map(float, in_string.strip('[]').split(',')))
                        assert len(in_floats) == 3
                        xyz_goal = in_floats
                        prnttmpl = coloredtext(50,255,50,'\n\tEndpoint Goal Input was [' + '{:.3f}, '*2 + '{:.3f}]')
                        self.get_logger().info(prnttmpl.format(*xyz_goal))
                    except:
                        retry = input('Bad Input. Try again? [y]/n: ')
                        if (retry=='y') or (retry=='Y') or (retry==''):
                            continue
                        else:
                            return
            
                # Compute Inverse Kinematics
                self.request_inverse_kinematics.endpoint = xyz_goal
                future = self.cli_inverse_kinematics.call_async(self.request_inverse_kinematics)
                rclpy.spin_until_future_complete(self, future)
            
                response = future.result()
                endpoint = response.endpoint
            
                # Check if the response is what you asked for, or something else
                if response.modified:
                    self.get_logger().info(coloredtext(255,0,0,'Unreachable Endpoint!'))
                    if not any(np.isnan(endpoint)):
                        go_anyway = input(coloredtext(50,255,50,'Move to nearest point [' + '{:.3f}, '*2 + '{:.3f}] - [y]/n?\n').format(*endpoint))
                        if len(go_anyway)==0:
                            go_anyway='y'
                    else:
                        go_anyway = 'n'
                
                    if not (go_anyway[0].upper() == 'Y') :
                        self.get_logger().info(coloredtext(255,0,0,'Not moving - try again.'))
                        continue
            
                # Preserve existing joint angles for wrist and gripper
                # Only update the first 3 joints from inverse kinematics
                new_ang_all = list(response.joint_angles)
                new_ang_all[3:] = self.ang_all[3:]  # Preserve forearm, wrist, and gripper joints
            
                # Update joint angles
                self.ang_all = new_ang_all
                self.ang_all[-1] = self.gripper_state  # Ensure gripper state is correct
            
                self.get_logger().info(coloredtext(50,255,50,'\n\tMoving to [' + '{:.3f}, '*2 + '{:.3f}]').format(*endpoint))
                self.get_logger().info(coloredtext(25,255,75,'\n\tAngles [' + '{:.3f}, '*5 + '{:.3f}]').format(*self.ang_all))
                self.send_joint_angles_desired()
            
            except Exception as e:
                self.get_logger().error(f'An error occurred: {str(e)}')
                retry = input('An error occurred. Try again? [y]/n: ')
                if (retry.lower() not in ['y', '']) :
                    return


    def send_joint_angles_desired(self):
        # Ensure gripper state is preserved when sending joint angles
        full_angles = np.append(self.ang_all, 0.)
        full_angles[6] = self.gripper_state  # Explicitly set gripper state
        self.joint_angles_desired_msg.position = full_angles
        self.joint_angles_desired_msg.header.stamp = self.get_clock().now().to_msg()
        self.pub_joint_angles_desired.publish(self.joint_angles_desired_msg)


def main(args=None):
    try:
        rclpy.init(args=args)
        endpoint_keyboard_instance = EndpointKeyboard()  
        endpoint_keyboard_instance.endpoint_requests()
       
    except KeyboardInterrupt:
        print("Exiting...")
    except:
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()