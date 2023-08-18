#!/usr/bin/env python3
import rospy
from std_msgs.msg import String
import json
import sys
import actionlib
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import PoseStamped
from franka_gripper.msg import GraspActionGoal, GraspGoal, GraspAction, MoveGoal, MoveAction, StopActionGoal, StopAction


# Class for moving Panda
class MovePanda:
    def __init__(self, piece_height, piece_width):
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()
        self.group = moveit_commander.MoveGroupCommander("panda_arm")
        self.display_trajectory_publisher = rospy.Publisher(
                                    '/move_group/display_planned_path',
                                    moveit_msgs.msg.DisplayTrajectory, queue_size=10)
        # self.group.set_planner_id("RRTstar")
        self.group.set_max_velocity_scaling_factor(0.6)
        self.group.set_max_acceleration_scaling_factor(0.3)

        self.start_pos = None
        self.goal_pos = None
        self.piece_height = piece_height
        self.piece_width = piece_width
        self.client_gripper = None
        # [-0.2209265405880759, -1.707389375161992, -0.03387857147275907, -2.768483632693613, 0.09437357893917296, 1.5371560325622804, 0.5672116772923153]
        # yukarıdan (0.05571041448074475, -1.7458552775053384, -0.05703916199225719, -2.7906102089561, -0.09018639426822503, 1.5974467511253858, 0.853530557214386) olmadı
        # yandan -0.13588635811678593, -1.4528482668414435, -0.8850022091893508, -2.678756275063777, -0.3791504966153038, 1.7860846366940417, -0.23471995383569405 olmadı
        # son [-0.341326954260207, -1.1988814659068092, -0.4588813986528743, -2.5964686106236083, 0.020460680315669704, 1.6782508972954784, -0.1153142092037532] olmadı
        # yukarıdan2 0.07725329711611768, -1.7278641462357827, -0.15132292361592398, -2.840492333280366, -0.1938036083049244, 1.6180341879542002, 0.895061891957528
        self.camera_pose =  [-0.2209265405880759, -1.707389375161992, -0.03387857147275907, -2.768483632693613, 0.09437357893917296, 1.5371560325622804, 0.5672116772923153]
        self.init_pose = (0.20569760583994678, -0.46903115825910924, -0.21801485168053691, -2.3468741699571547, -0.10967289843824174, 1.904154620806376, 0.8666427851311035)
        # self.camera_pose = (0.050316248142134444, -1.1722614542475915, -0.004582929329747745, -2.4842828935251484, -0.0387168858812915, 1.6610745452245075, 0.825098671177206)
        self.init_xyz = [0.431242278202267, -0.016861262207202146, 0.49237433009313425]
        self.current_pose = None

        self.box = PoseStamped()
        self.box.header.frame_id = self.robot.get_planning_frame()
        self.box.pose.position.x = 0.2
        self.box.pose.position.y = 0.0
        self.box.pose.position.z = -0.3
        self.scene.add_box("table",self.box,(2,2,0.6))

        self.not_move_camera_init = False
        self.not_move_init = False


    def set_pos(self, pos):
        self.start_pos = pos[0]
        self.goal_pos = pos[1]

    def move_camera_init(self):
        self.group.go(self.camera_pose, wait = True)
        # close the gripper for camera reasons
        self.move_gripper(0.09)

    def move_init(self):
        self.group.go(self.init_pose, wait = True)

    def move_pose(self, x, y, z):
        self.current_pose = self.group.get_current_joint_values()
        self.group.clear_pose_targets()

        print ("Generating plan")
        pose_target = geometry_msgs.msg.Pose()
        pose_target.orientation = self.group.get_current_pose().pose.orientation
        pose_target.position.x = x
        pose_target.position.y = y
        pose_target.position.z = z

        self.group.set_pose_target(pose_target)
        plan_success, plan1, planning_time, error_code = self.group.plan()
        print("Plan success: " + str(plan_success))

        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = self.robot.get_current_state()
        display_trajectory.trajectory.append(plan1)
        self.display_trajectory_publisher.publish(display_trajectory)
        self.group.execute(plan1)


    # we don't use this function
    def get_pos(self):
        # chess board dimensions and center coordinates
        side_len = 0.047
        center_x = 0.5153487764139318
        center_y = -0.029912709237291616
        center_z = 0.1672171928305743
        move_coord = []

        # Calculate the minimum and maximum coordinates for the chessboard
        min_x = center_x + (3.5 * side_len)
        min_y = center_y - (3.5 * side_len)
        z = center_z

        # Edit chess_table and add the square coordinate information using chess board dimensions 
        """ for row in range(8):
            for col in range(8): """
        row = 7
        col = 1
        x = min_x - (row * side_len)
        y = min_y + (col * side_len)
        self.start_pos = [x, y, z]

        row = 5
        col = 2
        x = min_x - (row * side_len)
        y = min_y + (col * side_len)
        self.goal_pos = [x, y, z]

        self.pick_and_place()
        

    def move_gripper(self, unit):
        # Creates the SimpleActionClient, passing the type of the action
        self.client_gripper = actionlib.SimpleActionClient('/franka_gripper/move', MoveAction)

        # Waits until the action server has started up and started
        # listening for goals.
        print("GRIPPER MOVEMENT")

        print("waiting for server")
        self.client_gripper.wait_for_server()

        # Creates a goal to send to the action server.
        print("creating goal")
        goal = MoveGoal(width=unit, speed=1.0)

        # Sends the goal to the action server.
        print("sending goal")
        self.client_gripper.send_goal(goal)

        # Waits for the server to finish performing the action.
        print("wait for result")
        self.client_gripper.wait_for_result()

        # Get the result
        result = self.client_gripper.get_result()
        print("Gripper movement result:", result)


    def grasp(self, width, speed):
        # Create the SimpleActionClient, passing the type of the action
        self.client_gripper = actionlib.SimpleActionClient('/franka_gripper/grasp', GraspAction)

        # Wait until the action server has started up and started listening for goals.
        print("Waiting for server")
        self.client_gripper.wait_for_server()

        # Create a goal to send to the action server.
        goal = GraspGoal()
        goal.width = width
        goal.epsilon.inner = 0.006
        goal.epsilon.outer = 0.006
        goal.speed = speed
        goal.force = 9.0

        # Create the action goal message
        action_goal = GraspActionGoal()
        action_goal.goal = goal
        
        # Sends the goal to the action server.
        print("sending goal")
        self.client_gripper.send_goal(goal)

        # Waits for the server to finish performing the action.
        print("wait for result")
        self.client_gripper.wait_for_result()

        # Get the result
        result = self.client_gripper.get_result()
        print("Gripper grasping result:", result)


    def pick_piece(self):
        print("picking up the piece")
        if (self.current_pose != self.init_pose and not self.not_move_init):
            self.move_init()

        self.move_gripper(0.03)
        top_pos = list(self.start_pos)

        # MOVING TO A POSITION ALIGNED WITH THE PIECE (HIGHER)
        top_pos[2] = self.start_pos[2] + self.piece_height * 4
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])
        print("moved to top position")
        
        # MOVING TO A POSITION FOR PICKING UP THE PIECE
        # edit 0.8 to find the correct position
        top_pos[2] = self.start_pos[2] + self.piece_height * 0.8
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])
        
        # CLOSE THE GRIPPER
        self.grasp(self.piece_width, 0.1)

        # MOVING TO A POSITION WITH THE PIECE (HIGHER)
        top_pos[2] = self.start_pos[2] + self.piece_height * 4
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])


    def drop_piece(self):
        print("dropping the piece")
        top_pos = list(self.goal_pos)

        # MOVING TO A POSITION WITHOUT THE PIECE (HIGHER)
        top_pos[2] = self.goal_pos[2] + self.piece_height * 4
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])

        # MOVING TO A POSITION FOR DROPPING THE PIECE
        # edit 0.8 to find the correct position
        top_pos[2] = self.goal_pos[2] + self.piece_height * 0.8
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])

        # DROP PIECE
        self.move_gripper(0.03)

        # MOVING TO A POSITION WITHOUT THE PIECE (HIGHER)
        top_pos[2] = self.goal_pos[2] + self.piece_height * 4
        # move to position
        self.move_pose(top_pos[0], top_pos[1], top_pos[2])


    def pick_and_place(self):
        # implement lowering the arm as well as closing the gripper in this function
        self.pick_piece()
        # implement moving the arm up as well as opening the gripper in this function
        self.drop_piece()

        if self.not_move_camera_init:
            print("finished first move")
        else:
            self.move_camera_init()

# global variables
global panda
panda = MovePanda(0.02, 0.008)

def control(data):
    global panda
    parsed_data = json.loads(data.data)

    move_coord = parsed_data["move"]
    panda.not_move_init = parsed_data["bool1"]
    panda.not_move_camera_init = parsed_data["bool2"]

    print(move_coord)
    
    panda.set_pos(move_coord)
    panda.pick_and_place()
    pub.publish("done")


def listener():
    rospy.Subscriber("game_controller/move", String, control)
    rospy.spin()


if __name__ == '__main__':
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('motion_controller', log_level=rospy.INFO, anonymous=True, disable_signals=True)
    pub = rospy.Publisher("motion_controller/robot_done_moving", String, queue_size=10)
    listener()
    # panda.move_init()
    #panda.get_pos()
    print("done")
