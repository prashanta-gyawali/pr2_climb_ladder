#! /usr/bin/python

import roslib
roslib.load_manifest('rospy')
roslib.load_manifest('geometry_msgs')
roslib.load_manifest('actionlib')
roslib.load_manifest('joint_trajectory_action')
roslib.load_manifest('kinematics_msgs')
roslib.load_manifest('tf')
roslib.load_manifest('pr2_controllers_msgs')
roslib.load_manifest('arm_navigation_msgs')

import rospy
import geometry_msgs
import actionlib
import kinematics_msgs
import tf
import pr2_controllers_msgs
import message_filters

from geometry_msgs.msg import *
from actionlib_msgs.msg import *
from kinematics_msgs.msg import *
from pr2_controllers_msgs.msg import *
from kinematics_msgs.srv import GetConstraintAwarePositionIK, GetConstraintAwarePositionIKRequest, GetKinematicSolverInfo, GetKinematicSolverInfoRequest, GetPositionIK, GetPositionIKRequest
from arm_navigation_msgs.srv import GetStateValidity,GetStateValidityRequest, SetPlanningSceneDiff, SetPlanningSceneDiffRequest
from std_msgs.msg import String

def MoveBody():
	# in order to move the base we need to  publish a geometry_msgs/Twist message on the base_controller/command topic
	pub = rospy.Publisher('base_controller/command', Twist)
	time.sleep(1)

	movement = Twist()
	# linear velocity is assigned 0.1
	movement.linear.x = -0.1
	start_time = rospy.get_rostime()
	while rospy.get_rostime() < start_time + rospy.Duration(0.2):
		pub.publish(movement)
		time.sleep(0.01)
	pub.publish(Twist())

def MoveTorso():
	client = actionlib.SimpleActionClient('torso_controller/position_joint_action', SingleJointPositionAction)
	client.wait_for_server()

	client.send_goal(SingleJointPositionGoal(position = 0.1))
	client.wait_for_result()
	if client.get_state() ==  GoalStatus.SUCCEEDED:
		print "Success moving the torso"
	

def MoveRightArmToJointAngles(joint_angles_list):
	client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
	client.wait_for_server()

	goal = JointTrajectoryGoal()
	goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
	goal.trajectory.joint_names = ["r_shoulder_pan_joint",
	                               "r_shoulder_lift_joint",
    	                           "r_upper_arm_roll_joint",
    	                           "r_elbow_flex_joint",
    	                           "r_forearm_roll_joint",
    	                           "r_wrist_flex_joint",
    	                           "r_wrist_roll_joint"]
	ind = 0
	
	point1 = trajectory_msgs.msg.JointTrajectoryPoint()
	goal.trajectory.points = [point1]
	#point1.positions = [-0.248, -0.34, -1.2, -1.577, -2.734, -1.462, -1.925]
	point1.positions = [joint_angles_list[0],
						joint_angles_list[1],
						joint_angles_list[2],
						joint_angles_list[3],
						joint_angles_list[4],
						joint_angles_list[5],
						joint_angles_list[6]]
	point1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	goal.trajectory.points[ind] = point1
	goal.trajectory.points[ind].time_from_start = rospy.Duration(1.0)

	client.send_goal(goal)
	client.wait_for_result()

def MoveLeftArmToJointAngles(joint_angles_list):
	client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
	client.wait_for_server()

	goal = JointTrajectoryGoal()
	goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(1.0)
	goal.trajectory.joint_names = ["l_shoulder_pan_joint",
	                               "l_shoulder_lift_joint",
    	                           "l_upper_arm_roll_joint",
    	                           "l_elbow_flex_joint",
    	                           "l_forearm_roll_joint",
    	                           "l_wrist_flex_joint",
    	                           "l_wrist_roll_joint"]
	ind = 0
	
	point1 = trajectory_msgs.msg.JointTrajectoryPoint()
	goal.trajectory.points = [point1]
	#point1.positions = [-0.248, -0.34, -1.2, -1.577, -2.734, -1.462, -1.925]
	point1.positions = [joint_angles_list[0],
						joint_angles_list[1],
						joint_angles_list[2],
						joint_angles_list[3],
						joint_angles_list[4],
						joint_angles_list[5],
						joint_angles_list[6]]
	point1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	goal.trajectory.points[ind] = point1
	goal.trajectory.points[ind].time_from_start = rospy.Duration(1.0)

	client.send_goal(goal)
	client.wait_for_result()

def MoveRightGripper(pose):
	client = actionlib.SimpleActionClient('r_gripper_controller/gripper_action', Pr2GripperCommandAction)
	client.wait_for_server()

	client.send_goal(Pr2GripperCommandGoal(
        Pr2GripperCommand(position = pose, max_effort = -1)))
	client.wait_for_result()

	result = client.get_result()
	did = []
	if client.get_state() != GoalStatus.SUCCEEDED:
		did.append("failed")
	else:
		if result.stalled: did.append("stalled")
		if result.reached_goal: did.append("reached goal")
	print ' and '.join(did)

def RotateRightGripper(jointList, rightGripperPose):
	client = actionlib.SimpleActionClient('r_arm_controller/joint_trajectory_action', JointTrajectoryAction)
	client.wait_for_server()

	goal = JointTrajectoryGoal()
	goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(5.0)
	goal.trajectory.joint_names = ["r_shoulder_pan_joint",
                                   "r_shoulder_lift_joint",
                                   "r_upper_arm_roll_joint",
                                   "r_elbow_flex_joint",
                                   "r_forearm_roll_joint",
                                   "r_wrist_flex_joint",
                                   "r_wrist_roll_joint"]
	ind = 0
	
	point1 = trajectory_msgs.msg.JointTrajectoryPoint()
	goal.trajectory.points = [point1]
	
	point1.positions = [jointList[0],
                        jointList[1],
                        jointList[2],
                        jointList[3],
                        jointList[4],
                        jointList[5],
                        rightGripperPose]
	point1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	goal.trajectory.points[ind] = point1
	goal.trajectory.points[ind].time_from_start = rospy.Duration(5.0)

	client.send_goal(goal)
	client.wait_for_result()

def MoveLeftGripper(pose):
	client = actionlib.SimpleActionClient('l_gripper_controller/gripper_action', Pr2GripperCommandAction)
	client.wait_for_server()
	
	client.send_goal(Pr2GripperCommandGoal(Pr2GripperCommand(position = pose, max_effort = -1)))
	client.wait_for_result()

	result = client.get_result();
	did = []
	if client.get_state() != GoalStatus.SUCCEEDED:
		did.append("failed")
	else:
		if result.stalled: did.append("stalled")
		if result.reached_goal: did.append("reached goal")
	print ' and '.join(did)

def RotateLeftGripper(jointList, leftGripperPose):
	client = actionlib.SimpleActionClient('l_arm_controller/joint_trajectory_action', JointTrajectoryAction)
	client.wait_for_server()

	goal = JointTrajectoryGoal()
	goal.trajectory.header.stamp = rospy.Time.now() + rospy.Duration(5.0)
	goal.trajectory.joint_names = ["l_shoulder_pan_joint",
                                   "l_shoulder_lift_joint",
                                   "l_upper_arm_roll_joint",
                                   "l_elbow_flex_joint",
                                   "l_forearm_roll_joint",
                                   "l_wrist_flex_joint",
                                   "l_wrist_roll_joint"]
	ind = 0
	
	point1 = trajectory_msgs.msg.JointTrajectoryPoint()
	goal.trajectory.points = [point1]
	
	point1.positions = [jointList[0],
                        jointList[1],
                        jointList[2],
                        jointList[3],
                        jointList[4],
                        jointList[5],
                        leftGripperPose]
	point1.velocities = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	goal.trajectory.points[ind] = point1
	goal.trajectory.points[ind].time_from_start = rospy.Duration(5.0)

	client.send_goal(goal)
	client.wait_for_result()

def FindCollisionAwareIKRight(point, orientation):
	rospy.wait_for_service('pr2_right_arm_kinematics/get_ik_solver_info')
	rospy.wait_for_service('pr2_right_arm_kinematics/get_constraint_aware_ik')

	query_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
	ik_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_constraint_aware_ik', GetConstraintAwarePositionIK)
	myik_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik', GetPositionIK)
	set_planning_scene_diff_client = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
	
	planning_scene_req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
	#set_planning_scene_diff_client.call(planning_scene_req)  
	
	if not(set_planning_scene_diff_client.call(planning_scene_req)):
		rospy.logwarn("cant get planning scene")
		sys.exit()

	# define the service messages
	request = kinematics_msgs.srv.GetKinematicSolverInfoRequest()
	response = query_client.call(request)

	# define the service messages
	gpik_req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()

	gpik_req.timeout = rospy.Duration(15.0)
	gpik_req.ik_request.ik_link_name = 'r_wrist_roll_link'

	gpik_req.ik_request.pose_stamped.header.frame_id = 'odom_combined'
	gpik_req.ik_request.pose_stamped.pose.position.x = point[0]
	gpik_req.ik_request.pose_stamped.pose.position.y = point[1]
	gpik_req.ik_request.pose_stamped.pose.position.z = point[2]

	gpik_req.ik_request.pose_stamped.pose.orientation.x = orientation[0]
	gpik_req.ik_request.pose_stamped.pose.orientation.y = orientation[1]
	gpik_req.ik_request.pose_stamped.pose.orientation.z = orientation[2]
	gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0
	gpik_req.ik_request.ik_seed_state.joint_state.position =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names

	for i in range (0, 7):
		gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0

	gpik_res = ik_client.call(gpik_req)
	if (gpik_res):
		if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS):
			for i in range (0, 7):
				rospy.loginfo("Joint: %s %f", gpik_res.solution.joint_state.name[i], gpik_res.solution.joint_state.position[i])
		else:
			rospy.logerr("Inverse kinematics failed")
	else:
		rospy.logerr("Inverse kinematics service call failed")	

	print 'all right m getting out of this function!!'
	return gpik_res.solution.joint_state.position

def FindCollisionAwareIKLeft(point, orientation):
	rospy.wait_for_service('pr2_left_arm_kinematics/get_ik_solver_info')
	rospy.wait_for_service('pr2_left_arm_kinematics/get_constraint_aware_ik')

	query_client = rospy.ServiceProxy('pr2_left_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
	ik_client = rospy.ServiceProxy('pr2_left_arm_kinematics/get_constraint_aware_ik', GetConstraintAwarePositionIK)
	myik_client = rospy.ServiceProxy('pr2_left_arm_kinematics/get_ik', GetPositionIK)
	set_planning_scene_diff_client = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
	
	planning_scene_req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
	#set_planning_scene_diff_client.call(planning_scene_req)  
	
	if not(set_planning_scene_diff_client.call(planning_scene_req)):
		rospy.logwarn("cant get planning scene")
		sys.exit()

	# define the service messages
	request = kinematics_msgs.srv.GetKinematicSolverInfoRequest()
	response = query_client.call(request)

	# define the service messages
	gpik_req = kinematics_msgs.srv.GetConstraintAwarePositionIKRequest()

	gpik_req.timeout = rospy.Duration(15.0)
	gpik_req.ik_request.ik_link_name = 'l_wrist_roll_link'

	gpik_req.ik_request.pose_stamped.header.frame_id = 'odom_combined'
	gpik_req.ik_request.pose_stamped.pose.position.x = point[0]
	gpik_req.ik_request.pose_stamped.pose.position.y = point[1]
	gpik_req.ik_request.pose_stamped.pose.position.z = point[2]

	gpik_req.ik_request.pose_stamped.pose.orientation.x = orientation[0]
	gpik_req.ik_request.pose_stamped.pose.orientation.y = orientation[1]
	gpik_req.ik_request.pose_stamped.pose.orientation.z = orientation[2]
	gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0
	gpik_req.ik_request.ik_seed_state.joint_state.position =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names

	for i in range (0, 7):
		gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0

	gpik_res = ik_client.call(gpik_req)
	if (gpik_res):
		if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS):
			for i in range (0, 7):
				rospy.loginfo("Joint: %s %f", gpik_res.solution.joint_state.name[i], gpik_res.solution.joint_state.position[i])
		else:
			rospy.logerr("Inverse kinematics failed")
	else:
		rospy.logerr("Inverse kinematics service call failed")	

	print 'all right m getting out of this function!!'
	return gpik_res.solution.joint_state.position

def FindIKRight(point, orientation):
	rospy.wait_for_service('pr2_right_arm_kinematics/get_ik_solver_info')
	rospy.wait_for_service('pr2_right_arm_kinematics/get_ik')

	query_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
	ik_client = rospy.ServiceProxy('pr2_right_arm_kinematics/get_ik', GetPositionIK)
	set_planning_scene_diff_client = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
	
	planning_scene_req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
	#set_planning_scene_diff_client.call(planning_scene_req)  
	
	if not(set_planning_scene_diff_client.call(planning_scene_req)):
		rospy.logwarn("cant get planning scene")
		sys.exit()
	
	# define the service messages
	request = kinematics_msgs.srv.GetKinematicSolverInfoRequest()
	response = query_client.call(request)

	# define the service messages
	gpik_req = kinematics_msgs.srv.GetPositionIKRequest()

	gpik_req.timeout = rospy.Duration(15.0)
	gpik_req.ik_request.ik_link_name = 'r_wrist_roll_link'

	gpik_req.ik_request.pose_stamped.header.frame_id = 'odom_combined'
	gpik_req.ik_request.pose_stamped.pose.position.x = point[0]
	gpik_req.ik_request.pose_stamped.pose.position.y = point[1]
	gpik_req.ik_request.pose_stamped.pose.position.z = point[2]

	gpik_req.ik_request.pose_stamped.pose.orientation.x = orientation[0]#1.0
	gpik_req.ik_request.pose_stamped.pose.orientation.y = orientation[1]#0.0
	gpik_req.ik_request.pose_stamped.pose.orientation.z = orientation[2]#-0.3
	gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0
	gpik_req.ik_request.ik_seed_state.joint_state.position =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names

	for i in range (0, 7):
		gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0

	gpik_res = ik_client.call(gpik_req)
	if (gpik_res):
		if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS):
			for i in range (0, 7):
				rospy.loginfo("Joint: %s %f", gpik_res.solution.joint_state.name[i], gpik_res.solution.joint_state.position[i])
		else:
			rospy.logerr("Inverse kinematics failed")
	else:
		rospy.logerr("Inverse kinematics service call failed")	

	print 'all right m getting out of this function!!'
	return gpik_res.solution.joint_state.position

def FindIKLeft(point, orientation):
	rospy.wait_for_service('pr2_left_arm_kinematics/get_ik_solver_info')
	rospy.wait_for_service('pr2_left_arm_kinematics/get_ik')

	query_client = rospy.ServiceProxy('pr2_left_arm_kinematics/get_ik_solver_info', GetKinematicSolverInfo)
	ik_client = rospy.ServiceProxy('pr2_left_arm_kinematics/get_ik', GetPositionIK)
	set_planning_scene_diff_client = rospy.ServiceProxy('/environment_server/set_planning_scene_diff', SetPlanningSceneDiff)
	
	planning_scene_req = arm_navigation_msgs.srv.SetPlanningSceneDiffRequest()
	#set_planning_scene_diff_client.call(planning_scene_req)  
	
	if not(set_planning_scene_diff_client.call(planning_scene_req)):
		rospy.logwarn("cant get planning scene")
		sys.exit()
	
	# define the service messages
	request = kinematics_msgs.srv.GetKinematicSolverInfoRequest()
	response = query_client.call(request)

	# define the service messages
	gpik_req = kinematics_msgs.srv.GetPositionIKRequest()

	gpik_req.timeout = rospy.Duration(15.0)
	gpik_req.ik_request.ik_link_name = 'l_wrist_roll_link'

	gpik_req.ik_request.pose_stamped.header.frame_id = 'odom_combined'
	gpik_req.ik_request.pose_stamped.pose.position.x = point[0]
	gpik_req.ik_request.pose_stamped.pose.position.y = point[1]
	gpik_req.ik_request.pose_stamped.pose.position.z = point[2]

	gpik_req.ik_request.pose_stamped.pose.orientation.x = orientation[0]
	gpik_req.ik_request.pose_stamped.pose.orientation.y = orientation[1]
	gpik_req.ik_request.pose_stamped.pose.orientation.z = orientation[2]
	gpik_req.ik_request.pose_stamped.pose.orientation.w = 1.0
	gpik_req.ik_request.ik_seed_state.joint_state.position =  [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
	gpik_req.ik_request.ik_seed_state.joint_state.name = response.kinematic_solver_info.joint_names

	for i in range (0, 7):
		gpik_req.ik_request.ik_seed_state.joint_state.position[i] = (response.kinematic_solver_info.limits[i].min_position + response.kinematic_solver_info.limits[i].max_position)/2.0

	gpik_res = ik_client.call(gpik_req)
	if (gpik_res):
		if (gpik_res.error_code.val == gpik_res.error_code.SUCCESS):
			for i in range (0, 7):
				rospy.loginfo("Joint: %s %f", gpik_res.solution.joint_state.name[i], gpik_res.solution.joint_state.position[i])
		else:
			rospy.logerr("Inverse kinematics failed")
	else:
		rospy.logerr("Inverse kinematics service call failed")	

	print 'all right m getting out of this function!!'
	return gpik_res.solution.joint_state.position

def GraspRight(point, orientation):
	# open the gripper
	MoveRightGripper(0.07)	

	# first go to a pre grasp position
	temp_point_right = list(point)   # to avoid change to the list in main 
	temp_point_right[0] = temp_point_right[0] - 0.2
	print temp_point_right[0]
	rt_arm_angles = FindIKRight(temp_point_right, orientation)
	MoveRightArmToJointAngles(rt_arm_angles)

	# then go to the grasp position
	temp_point_right[0] = temp_point_right[0] + 0.07
	print temp_point_right[0]
	rt_arm_angles = FindIKRight(temp_point_right, orientation)
	MoveRightArmToJointAngles(rt_arm_angles)

	# then close the gripper
	MoveRightGripper(0.049)

def GraspLeft(point, orientation):	
	# open the gripper
	MoveLeftGripper(0.07)	

	# first go to a pre grasp position
	temp_point_left = list(point) 	# to avoid change to the list in main
	temp_point_left[0] = temp_point_left[0] - 0.2
	print temp_point_left[0]
	lt_arm_angles = FindIKLeft(temp_point_left, orientation)
	MoveLeftArmToJointAngles(lt_arm_angles)

	# then go to the grasp position
	temp_point_left[0] = temp_point_left[0] + 0.07
	print temp_point_left[0]
	lt_arm_angles = FindIKLeft(temp_point_left, orientation)
	MoveLeftArmToJointAngles(lt_arm_angles)

	# then close the gripper
	MoveLeftGripper(0.049)	

def InitializeRight(point, orientation):
	# open the gripper
	MoveRightGripper(0.08)

	# then go to the previous pre grasp position
	temp_point_right = list(point)
	print point
	print temp_point_right[0]
	temp_point_right[0] = temp_point_right[0] - 0.3
	rt_arm_angles = FindIKRight(temp_point_right, orientation)
	MoveRightArmToJointAngles(rt_arm_angles)

def InitializeLeft(point, orientation):
	# open the gripper
	MoveLeftGripper(0.08)

	# then go to the previous pre grasp position
	temp_point_left = list(point)
	temp_point_left[0] = temp_point_left[0] - 0.3
	lt_arm_angles = FindIKLeft(temp_point_left, orientation)	
	MoveLeftArmToJointAngles(lt_arm_angles)

def genericFunction(centroid_rung1_kinect, centroid_rung2_kinect):
	rospy.loginfo("x y z: %f %f %f", centroid_rung1_kinect.point.x, centroid_rung1_kinect.point.y, centroid_rung1_kinect.point.z)
	rospy.loginfo("x y z: %f %f %f", centroid_rung2_kinect.point.x, centroid_rung2_kinect.point.y, centroid_rung2_kinect.point.z)
	
	rt_arm_points = [centroid_rung1_kinect.point.x, centroid_rung1_kinect.point.y-0.15, centroid_rung1_kinect.point.z]
	lt_arm_points = [centroid_rung1_kinect.point.x, centroid_rung1_kinect.point.y+0.15, centroid_rung1_kinect.point.z]					 
	
	'''
	rt_arm_points = [0.68852, -0.2183, 1.21]
	lt_arm_points = [0.68852, 0.0817, 1.21] 
	'''

	orientation = [1.0, 0.0, 0.0]	
	rt_arm_angles = []
	lt_arm_angles = []
	
	temp_point_left = []
	temp_point_right = []
	index = 0
	mode = 0
	temp = 0

	GraspLeft(lt_arm_points, orientation)
	GraspRight(rt_arm_points, orientation)	

	MoveLeftArmToJointAngles([0.519860, 0.503715, 1.55, -1.4738, -2.618218, -0.955526, 3.106722])		
	MoveRightArmToJointAngles([-0.520718, 0.504227, -1.55, -1.475069, 2.618079, -0.956160, 0.034233])

	
if __name__ == '__main__':
	rospy.init_node('climb', anonymous=True)
	rung1_sub = message_filters.Subscriber('Centroid_Odom_Combined_Rung1', PointStamped)
	rung2_sub = message_filters.Subscriber('Centroid_Odom_Combined_Rung2', PointStamped)
	
	ts = message_filters.TimeSynchronizer([rung1_sub, rung2_sub], 10)
	ts.registerCallback(genericFunction)
	rospy.spin()
