#! /usr/bin/env python

import rospy
import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import actionlib

import rospkg
import yaml
import os
import time
from std_srvs.srv import Empty


import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

import tf2_ros
import tf2_msgs.msg


from gazebo_ros_link_attacher.srv import Attach, AttachRequest, AttachResponse
from pkg_vb_sim.srv import vacuumGripper
from pkg_vb_sim.srv import vacuumGripperRequest
from pkg_vb_sim.srv import vacuumGripperResponse

from pkg_vb_sim.srv import conveyorBeltPowerMsg#to be used in this 
from pkg_vb_sim.srv import conveyorBeltPowerMsgRequest
from pkg_vb_sim.srv import conveyorBeltPowerMsgResponse

from pkg_vb_sim.srv import ConveyorBeltControl
from pkg_vb_sim.srv import ConveyorBeltControlRequest
from pkg_vb_sim.srv import ConveyorBeltControlResponse
from hrwros_gazebo.msg import LogicalCameraImage
from pkg_vb_sim.msg import Model

class ur5_moveit:

    # Constructor
    def __init__(self,arg_robot_name):
	self._robot_ns = '/'  + arg_robot_name
        self.state=0
    	
	self.colours=[]
   	self.packages=['packagen00','packagen01','packagen02','packagen10','packagen11','packagen12','packagen20','packagen21','packagen22',
          'packagen30','packagen31','packagen32']
	self.pkg_clr=dict(zip(self.packages,self.colours))
        self.final_clr=''
	#self.gripper=rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
	self.logical_camera=rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage, self.Logical_camera_callback,queue_size=1)
	self.conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)#to activate conveyer belt
        self._planning_group = "manipulator"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)

	self._robot = moveit_commander.RobotCommander(robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._scene = moveit_commander.PlanningSceneInterface(ns=self._robot_ns)
        self._group = moveit_commander.MoveGroupCommander(self._planning_group, robot_description= self._robot_ns + "/robot_description", ns=self._robot_ns)
        self._display_trajectory_publisher = rospy.Publisher( self._robot_ns + '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
        
	#subscribe to 2D camera:
	self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/eyrc/vb/camera_1/image_raw", Image,self.callback_for_2D_camera)	
	self._box_name = ''


        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''
        #getting current robot pose to add box
	self._curr_state = self._robot.get_current_state()

	#defining path to save trajevtory
 	rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}".format(self._file_path) )

        
        
        self._planning_frame = self._group.get_planning_frame()
        self._eef_link = self._group.get_end_effector_link()
        self._group_names = self._robot.get_group_names()
 	
        rospy.loginfo('\033[94m' + " >>> Init done." + '\033[0m')
	rospy.loginfo(
            '\033[94m' + "Planning Frame name: {}".format(self._planning_frame) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "End Effector Link: {}".format(self._eef_link) + '\033[0m')
        rospy.loginfo(
            '\033[94m' + "Group Names: {}".format(self._group_names) + '\033[0m')

        rospy.loginfo('\033[94m' + " >>> Ur5Moveit init done." + '\033[0m')

    def get_dominant_colour(self, arg_img):
	    # setting values for base colors 
	    b = arg_img[:, :, 0] 
	    g = arg_img[:, :, 1] 
	    r = arg_img[:, :, 2] 
	  
	    # computing the mean 
	    b_mean = np.mean(b) 
	    g_mean = np.mean(g) 
	    r_mean = np.mean(r) 
	    #print("following are the means",b_mean,g_mean,r_mean)
	    # displaying the most prominent color 
	    if (g_mean > r_mean and g_mean > b_mean): 
		#print("green")
		return 'green' 
	    elif ((g_mean +2) < r_mean and r_mean > b_mean): 
		#print("red")
		return 'red'
	    elif (b_mean > r_mean and g_mean < b_mean): 
		
		return 'blue'
	    else: 
		#print("yellow")
		return 'yellow'


    def callback_for_2D_camera(self,data):
	    try:
	      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
	    except CvBridgeError as e:
	      rospy.logerr(e)
  	    
	    (rows,cols,channels) = cv_image.shape
	    image = cv_image
	    self.colours=[]
	    # Resize a 720x1280 image to 360x640 to fit it on the screen
	    resized_image = cv2.resize(image, (720/2, 1280/2)) 
	    #print(resized_image[:,:,1])
	    cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
	    for i in range(150,470,80):
    		for j in range(0,320,120):
			self.colours.append(self.get_dominant_colour(resized_image[i:(i+80),j:(j+120),:]))
			#rospy.loginfo(self.get_dominant_colour(resized_image[i:(i+80),j:(j+120):]))
 	    
	    #rospy.loginfo(self.colours)
	    cv2.waitKey(3)

    def clear_octomap(self):
	clear_octomap_service_proxy = rospy.ServiceProxy(self._robot_ns + "/clear_octomap", Empty)
	return clear_octomap_service_proxy()


    def Logical_camera_callback(self,msg):
	#print(len(msg.models))
        #print(msg.models[0].type)
        #print(msg)
	#print(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
    	if len(msg.models) > 0 :
		if msg.models[-1].type in self.packages:
			#print("ininininininininininininiiininininnininininininiiininininininininininininnininininininiinininninininin")
			if round(msg.models[-1].pose.position.y,1)==0.0:
				
				rospy.loginfo("33333333333333333333333333333333333333333333333333333333333333333333333333")
				self.state=1
				model=str(msg.models[-1].type)
				self.final_clr=self.pkg_clr[model]
				print(self.final_clr)
				
  	else:
		self.state=0

    def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
	    # Copy class variables to local variables to make the web tutorials more clear.
	    # In practice, you should use the class variables directly unless you have a good
	    # reason not to.
	    box_name = self._box_name
	    scene = self._scene

	    ## BEGIN_SUB_TUTORIAL wait_for_scene_update
	    ##
	    ## Ensuring Collision Updates Are Received
	    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	    ## If the Python node dies before publishing a collision object update message, the message
	    ## could get lost and the box will not appear. To ensure that the updates are
	    ## made, we wait until we see the changes reflected in the
	    ## ``get_attached_objects()`` and ``get_known_object_names()`` lists.
	    ## For the purpose of this tutorial, we call this function after adding,
	    ## removing, attaching or detaching an object in the planning scene. We then wait
	    ## until the updates have been made or ``timeout`` seconds have passed
	    start = rospy.get_time()
	    seconds = rospy.get_time()
	    while (seconds - start < timeout) and not rospy.is_shutdown():
	      # Test if the box is in attached objects
	      attached_objects = scene.get_attached_objects([box_name])
	      is_attached = len(attached_objects.keys()) > 0

	      # Test if the box is in the scene.
	      # Note that attaching the box will remove it from known_objects
	      is_known = box_name in scene.get_known_object_names()

	      # Test if we are in the expected state
	      if (box_is_attached == is_attached) and (box_is_known == is_known):
		return True

	      # Sleep so that we give other threads time on the processor
	      rospy.sleep(0.1)
	      seconds = rospy.get_time()

	    # If we exited the while loop without returning then we timed out
	    return False
	    ## END_SUB_TUTORIAL
   
		 
    def add_box_to_scene(self, timeout=4):
    	# Copy class variables to local variables to make the web tutorials more clear.
    	# In practice, you should use the class variables directly unless you have a good
	# reason not to.
	print("within the functionLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLLL")
    	box_name = self._box_name
    	scene = self._scene
        curr_pose=self._group.get_current_pose().pose
    	## BEGIN_SUB_TUTORIAL add_box
    	##
    	## Adding Objects to the Planning Scene
    	## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    	## First, we will create a box in the planning scene between the fingers:
    	box_pose = geometry_msgs.msg.PoseStamped()
    	box_pose.header.frame_id = "world"
    	box_pose.pose.orientation.w = 1.0
        box_pose.pose.position.z = curr_pose.position.z 
        box_pose.pose.position.x = curr_pose.position.x
        box_pose.pose.position.y = curr_pose.position.y-0.20
	box_name = "package$1"
    	scene.add_box(box_name,box_pose, size=(0.15, 0.15, 0.15))
	print("done all the boxing")
	
    	## END_SUB_TUTORIAL
    	# Copy local variables back to class variables. In practice, you should use the class
    	# variables directly unless you have a good reason not to.
    	self._box_name=box_name
	return self.wait_for_state_update(box_is_known=True, timeout=timeout)




    def attach_box(self, timeout=4):
	    # Copy class variables to local variables to make the web tutorials more clear.
	    # In practice, you should use the class variables directly unless you have a good
	    # reason not to.
	    box_name = self._box_name
	    robot = self._robot
	    scene = self._scene
	    eef_link = self._eef_link
	    group_names = self._group_names

	    ## BEGIN_SUB_TUTORIAL attach_object
	    ##
	    ## Attaching Objects to the Robot
	    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	    ## Next, we will attach the box to the Panda wrist. Manipulating objects requires the
	    ## robot be able to touch them without the planning scene reporting the contact as a
	    ## collision. By adding link names to the ``touch_links`` array, we are telling the
	    ## planning scene to ignore collisions between those links and the box. For the Panda
	    ## robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
	    ## you should change this value to the name of your end effector group name.
	    grasping_group = 'manipulator'
	    touch_links = robot.get_link_names(group=grasping_group)
	    scene.attach_box(eef_link, box_name, touch_links=touch_links)
	    ## END_SUB_TUTORIAL

	    # We wait for the planning scene to update.
	    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)

    def detach_box(self, timeout=4):
	    # Copy class variables to local variables to make the web tutorials more clear.
	    # In practice, you should use the class variables directly unless you have a good
	    # reason not to.
	    box_name = self._box_name
	    scene = self._scene
	    eef_link = self._eef_link

	    ## BEGIN_SUB_TUTORIAL detach_object
	    ##
	    ## Detaching Objects from the Robot
	    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	    ## We can also detach and remove the object from the planning scene:
	    scene.remove_attached_object(eef_link, name=box_name)
	    ## END_SUB_TUTORIAL

	    # We wait for the planning scene to update.
	    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)

    def remove_box(self, timeout=4):
	    # Copy class variables to local variables to make the web tutorials more clear.
	    # In practice, you should use the class variables directly unless you have a good
	    # reason not to.
	    box_name = self._box_name
	    scene = self.scene

	    ## BEGIN_SUB_TUTORIAL remove_object
	    ##
	    ## Removing Objects from the Planning Scene
	    ## ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	    ## We can remove the box from the world.
	    scene.remove_world_object(box_name)

	    ## **Note:** The object must be detached before we can remove it from the world
	    ## END_SUB_TUTORIAL

	    # We wait for the planning scene to update.
	    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)

	   

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []
	self.start=0
        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)
 	#self._computed_plan = self._group.plan()
        # 3. Create a New waypoint
        wpose = geometry_msgs.msg.Pose()
        wpose.position.x = waypoints[0].position.x + (trans_x)  
        wpose.position.y = waypoints[0].position.y + (trans_y)  
        wpose.position.z = waypoints[0].position.z + (trans_z)
        # This to keep EE parallel to Ground Plane
        wpose.orientation.x = -0.5
        wpose.orientation.y = -0.5
        wpose.orientation.z = 0.5
        wpose.orientation.w = 0.5


        # 4. Add the new waypoint to the list of waypoints
        waypoints.append(copy.deepcopy(wpose))

        
        # 5. Compute Cartesian Path connecting the waypoints in the list of waypoints
        (plan, fraction) = self._group.compute_cartesian_path(
            waypoints,   # waypoints to follow
            0.01,        # Step Size, distance between two adjacent computed waypoints will be 1 cm
            0.0)         # Jump Threshold
        rospy.loginfo("Path computed successfully. Moving the arm.")

        # The reason for deleting the first two waypoints from the computed Cartisian Path can be found here,
        
        num_pts = len(plan.joint_trajectory.points)
        if (num_pts >= 3):
            del plan.joint_trajectory.points[0]
            del plan.joint_trajectory.points[1]

        # 6. Make the arm follow the Computed Cartesian Path
        self._group.execute(plan)

    def set_joint_angles(self, arg_list_joint_angles):

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Current Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        self._group.set_joint_value_target(arg_list_joint_angles)
	self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        if (flag_plan == True):
	    #pass
            rospy.loginfo(
                '\033[94m' + ">>> set_joint_angles() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> set_joint_angles() Failed." + '\033[0m')

        return flag_plan

    def hard_set_joint_angles(self, arg_list_joint_angles, arg_max_attempts):

        number_attempts = 0
        flag_success = False
        
        while ( (number_attempts <= arg_max_attempts) and  (flag_success is False) ):
            number_attempts += 1
            flag_success = self.set_joint_angles(arg_list_joint_angles)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # self.clear_octomap()

    def print_joint_angles(self):
        list_joint_values = self._group.get_current_joint_values()
	print(math.radians(list_joint_values[0]),math.radians(list_joint_values[1]),math.radians(list_joint_values[2]),
              math.radians(list_joint_values[3]),math.radians(list_joint_values[4]),math.radians(list_joint_values[5]))
        rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.degrees(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.degrees(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.degrees(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.degrees(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.degrees(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.degrees(list_joint_values[5])) +
                      '\033[0m')
    	rospy.loginfo('\033[94m' + "\nJoint Values: \n\n" +
                      "ur5_shoulder_pan_joint: {}\n".format(math.radians(list_joint_values[0])) +
                      "ur5_shoulder_lift_joint: {}\n".format(math.radians(list_joint_values[1])) +
                      "ur5_elbow_joint: {}\n".format(math.radians(list_joint_values[2])) +
                      "ur5_wrist_1_joint: {}\n".format(math.radians(list_joint_values[3])) +
                      "ur5_wrist_2_joint: {}\n".format(math.radians(list_joint_values[4])) +
                      "ur5_wrist_3_joint: {}\n".format(math.radians(list_joint_values[5])) +
                      '\033[0m')
	
    def go_to_pose(self, arg_pose):
	#self.print_joint_angles()
        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
	self.state=0
        self._group.set_pose_target(arg_pose)
	self._computed_plan = self._group.plan()
        flag_plan = self._group.go(wait=True)  # wait=False for Async Move

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Final Pose:" + '\033[0m')
        rospy.loginfo(pose_values)

        list_joint_values = self._group.get_current_joint_values()
        rospy.loginfo('\033[94m' + ">>> Final Joint Values:" + '\033[0m')
        rospy.loginfo(list_joint_values)

        if (flag_plan == True):
            rospy.loginfo('\033[94m' + ">>> go_to_pose() Success" + '\033[0m')
        else:
            rospy.logerr(
                '\033[94m' + ">>> go_to_pose() Failed. Solution for Pose not Found." + '\033[0m')
	
        return flag_plan

    def moveit_play_planned_path_from_file(self, arg_file_path, arg_file_name):
        file_path = arg_file_path + arg_file_name
        
        with open(file_path, 'r') as file_open:
            loaded_plan = yaml.load(file_open)
        
        ret = self._group.execute(loaded_plan)
        # rospy.logerr(ret)
        return ret

    
    def moveit_hard_play_planned_path_from_file(self, arg_file_path, arg_file_name, arg_max_attempts):
        number_attempts = 0
        flag_success = False

        while ( (number_attempts <= arg_max_attempts) and (flag_success is False) ):
            number_attempts += 1
            flag_success = self.moveit_play_planned_path_from_file(arg_file_path, arg_file_name)
            rospy.logwarn("attempts: {}".format(number_attempts) )
            # # self.clear_octomap()

    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
        '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

def main():
    rospy.init_node("node_task_4")
    rospy.sleep(5)
    gripper_ur5_1=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
    rospy.sleep(2)
    gripper_ur5_2=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
    
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
    ur5_1=ur5_moveit("ur5_1")
    ur5_2=ur5_moveit("ur5_2")
    ur5_1.pkg_clr=dict(zip(ur5_1.packages,ur5_1.colours))
    ur5_2.pkg_clr=dict(zip(ur5_2.packages,ur5_2.colours))
    rospy.loginfo(ur5_1.pkg_clr)
    ur5_home_pose = geometry_msgs.msg.Pose()
    ur5_home_pose.position.x = -0.8
    ur5_home_pose.position.y = 0
    ur5_home_pose.position.z = 1.04+ vacuum_gripper_width + (box_length/2)

    # This to keep EE parallel to Ground Plane
    ur5_home_pose.orientation.x = -0.5
    ur5_home_pose.orientation.y = -0.5
    ur5_home_pose.orientation.z = 0.5
    ur5_home_pose.orientation.w = 0.5
    

    ur5_pose_1 = geometry_msgs.msg.Pose()##for yellow bin
    ur5_pose_1.position.x =  0.598978209204
    ur5_pose_1.position.y = 0.213119414841
    ur5_pose_1.position.z = 1.13093406061
    ur5_pose_1.orientation.x = -0.000263851030942
    ur5_pose_1.orientation.y =0.705635523536
    ur5_pose_1.orientation.z =-0.708574925692
    ur5_pose_1.orientation.w = 0.000113962676622
    yellow_bin=[0.1368598944515913, -2.4426435995436195, -1.0170742065304177, -1.2521814115466077, 1.5699758209984171, 0.1364107680083091]



    ur5_pose_2 = geometry_msgs.msg.Pose()#green
    ur5_pose_2.position.x =-0.101329840758
    ur5_pose_2.position.y =-0.545843469289
    ur5_pose_2.position.z = 1.32248663163
    ur5_pose_2.orientation.x = -0.499664561444
    ur5_pose_2.orientation.y = -0.499953712369
    ur5_pose_2.orientation.z = 0.499851129103
    ur5_pose_2.orientation.w =0.50053017917



    ur5_pose_3 = geometry_msgs.msg.Pose()#red
    ur5_pose_3.position.x = 0.0342009771042#0.0315403281775
    ur5_pose_3.position.y = 0.700428334843#0.524706390977
    ur5_pose_3.position.z = 1.24463137182#1.50694485448
    ur5_pose_3.orientation.x = -0.500323940784#0.000265245021037
    ur5_pose_3.orientation.y = -0.500121038077#0.999999954882
    ur5_pose_3.orientation.z =0.500148199501#4.36880764989e-05
    ur5_pose_3.orientation.w = 0.49940632764#0.000134058645669
    red=[1.3015899410950142, 0.020741959457769354, -1.3618730223587283, -1.8005522920161106, -1.3021204204792731, 3.141348636613179]

    #rospy.loginfo(ur5_1._robot.get_current_state())

    package00 = geometry_msgs.msg.Pose()
    package00.position.x = 0.253703468186#0.251703468186
    package00.position.y = -0.19250000000
    package00.position.z = 1.89230063157
    package00.orientation.x = -0.000762232737515
    package00.orientation.y = -0.0147033855436
    package00.orientation.z = -0.998624001953
    package00.orientation.w = 0.0503322180951
    #-0.12203503728281362
    pkg_00=[-1.0038403010423727, -1.1353167358787388, -0.12203503728281362, -1.9168408548434082, -2.036075328616967, -0.01697799671428335]
    
    
    package01 = geometry_msgs.msg.Pose()
    package01.position.x = -0.0283245727752#-0.0543230559649
    package01.position.y = -0.20253186306#-0.19737303449
    package01.position.z = 1.9063361279#1.9329594746
    package01.orientation.x = 0.997674869012#-0.99291891021
    package01.orientation.y = 0.025668070636#-0.000415898754279
    package01.orientation.z = 0.00509741411347#-0.112813555154
    package01.orientation.w = 0.0629287077721#0.0372151386965

    

    pkg01=[1.9948542144893544, -2.0290575880282393, 0.46413353069543106, -1.4412987936575092, 1.2008496734095013, 3.099382417141065]

    '''position.x: 0.0044454935866
    position.y: -0.191057632201
    position.z: 1.93526999543

    roll: -3.14136545799
    pitch: -0.000189493320503
    yaw: -0.117409530769



    orientation.x: -0.998277359773
    orientation.y: 0.0586710412631
    orientation.z: -0.000101248348497
    orientation.w: 0.000107843225513

    [-2.1554254801716812, -1.3411749144461078, -0.07575848906966254, -1.7244047815577108, -1.103576674154585, -3.141517774324176]'''
    
    package02 = geometry_msgs.msg.Pose()
    package02.position.x = -0.254400060761
    package02.position.y = -0.161175272769
    package02.position.z = 1.8933862352
    package02.orientation.x = -0.269958016084
    package02.orientation.y = 0.0752309984613
    package02.orientation.z = 0.959692720748
    package02.orientation.w = 0.0212802294563

    

    #pkh02[0.9356012924204853, -1.8897342284602878, -0.10724073942009227, -0.9883265622302781, 2.1182403659935964, 0.6357526387511543]

    ''''ur5_1.go_to_pose(package00)
    ur5_1.hard_set_joint_angles(pose_home, 5)
    file_name = 'zeroooooo_to_pkggggggg00000_.yaml'
    file_path = ur5_1._file_path + file_name
    
    with open(file_path, 'w') as file_save:
    	yaml.dump(ur5_1._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    
    #rospy.sleep(5)
    
    #ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, 'zero_to_home.yaml', 5)
    pose_home = [0.13687080631389392, -2.4426565911031037, 
                     -1.017030652792866, -1.2521183124152495, 1.56992755139669, 0.13632543489507842]'''
    '''ur5_2.go_to_pose(ur5_pose_3)
    file_name = 'red_to_bin.yaml'
    file_path = ur5_2._file_path + file_name
    
    with open(file_path, 'w') as file_save:
    	yaml.dump(ur5_2._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    
    ur5_2.hard_set_joint_angles(pose_home, 5)
    
    file_name = 'red_bin_to_home.yaml'
    file_path = ur5_2._file_path + file_name
    
    with open(file_path, 'w') as file_save:
    	yaml.dump(ur5_2._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    ###################
    ur5_2.go_to_pose(ur5_pose_2)
    file_name = 'green_to_bin.yaml'
    file_path = ur5_2._file_path + file_name
    
    with open(file_path, 'w') as file_save:
    	yaml.dump(ur5_2._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    
    ur5_2.hard_set_joint_angles(pose_home, 5)

    file_name = 'green_bin_to_home.yaml'
    file_path = ur5_2._file_path + file_name
    
    with open(file_path, 'w') as file_save:
    	yaml.dump(ur5_2._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )

    ###########################
    ur5_2.go_to_pose(ur5_pose_1)
    file_name = 'yellow_to_bin.yaml'
    file_path = ur5_2._file_path + file_name
    
    with open(file_path, 'w') as file_save:
    	yaml.dump(ur5_2._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    
    ur5_2.hard_set_joint_angles(pose_home, 5)
    file_name = 'yellow_bin_to_home.yaml'
    file_path = ur5_2._file_path + file_name
    
    with open(file_path, 'w') as file_save:
    	yaml.dump(ur5_2._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )'''
    ###################
   
    ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, 'zero_to_home.yaml', 5)
    
    pose_home = [0.13687080631389392, -2.4426565911031037, 
                     -1.017030652792866, -1.2521183124152495, 1.56992755139669, 0.13632543489507842]
    #ur5_1.hard_set_joint_angles(pkg_00, 5)
    ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'zero_to_pkg00.yaml', 5)
    
    rospy.loginfo(ur5_1.pkg_clr)
    #ur5_1.go_to_pose(package00)
    rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
    #ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'zero_to_pkg00.yaml', 5)
    #ur5_1.go_to_pose(ur5_home_pose)
   
    #ur5_1.add_box_to_scene()
    #pick_00=gripper(True)
    gripper_ur5_1(True)
    ur5_1.add_box_to_scene()
    
    ur5_1.attach_box()
    rospy.logwarn("1. Playing pkg00 to home Trajectory File")
    ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'pkg00_to_home.yaml', 5)
    gripper_ur5_1(False)
    ur5_1.detach_box()
    ur5_1.conveyor(100)
    rospy.logwarn("1. Playing pkg00 to home Trajectory File")
    while(ur5_1.state==0):
	continue
    
    ur5_1.conveyor(0)
    #trajectory=ur5_2.final_clr+'_to_bin'
    #rospy.loginfo(trajectory)
    gripper_ur5_2(True)
    ur5_2.add_box_to_scene()
    ur5_2.attach_box()
    ur5_2.go_to_pose(ur5_pose_2)
    gripper_ur5_2(False)
    #ur5_1.go_to_pose(ur5_home_pose)
    #ur5_1.hard_set_joint_angles(pose_home, 5)
    
    #ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, 'pkg00_to_home.yaml', 5)
    
    '''file_name = 'home_to_bin.yaml'
    file_path = ur5_1._file_path + file_name
    
    with open(file_path, 'w') as file_save:
    	yaml.dump(ur5_1._computed_plan, file_save, default_flow_style=True)
    
    rospy.loginfo( "File saved at: {}".format(file_path) )
    #rospy.sleep(2)
    #rospy.logwarn("1. Playing AllZeros to Pose#1 Trajectory File")
    #ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'zero_to_home1.yaml', 5)
    rospy.spin()'''


if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException:
        pass
