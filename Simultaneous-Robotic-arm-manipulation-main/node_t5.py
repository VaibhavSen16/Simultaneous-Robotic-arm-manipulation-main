#! /usr/bin/env python

import rospy
import sys
import copy
import math
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import requests
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import actionlib
#import threading
from threading import Thread
import rospkg
import yaml
import os
import time
from std_srvs.srv import Empty


import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
#from sensor_msgs.msg import JointState
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


from pkg_ros_iot_bridge.msg import msgRosIotAction      # Message Class that is used by ROS Actions internally
from pkg_ros_iot_bridge.msg import msgRosIotGoal        # Message Class that is used for Goal Messages
from pkg_ros_iot_bridge.msg import msgRosIotResult      # Message Class that is used for Result Messages
from pkg_ros_iot_bridge.msg import msgMqttSub




class ur5_moveit(Thread):

    # Constructor
    def __init__(self,arg_robot_name):
        Thread.__init__(self)
	self._robot_ns = '/'  + arg_robot_name
        self.state=0
    	#self.pub=rospy.Publisher(self._robot_ns + '/ur5_1/joint_states, Twist, queue_size=1)
	self.colours=[]
	self.gripper_ur5_1=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
        #rospy.sleep(2)
        self.gripper_ur5_2=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
   	self.packages=['pkg00','pkg01','pkg02','pkg10','pkg11','pkg12','pkg20','pkg21','pkg22','pkg30','pkg31','pkg32']
	self.pkg_clr=dict(zip(self.packages,self.colours))
        self.final_clr=''
    	self._orders={}#to store orders coming from ros-iot bridge
	self.orderID=[]##to store ordersID in sorted order coming from ros-iot bridge
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
  	#self.orders_sub= rospy.Subscriber("/eyrc/vb/vaibhavs/orders",) 
	
	self._box_name = ''


        # Attribute to store computed trajectory by the planner	
        self._computed_plan = ''
        #getting current robot pose to add box
	self._curr_state = self._robot.get_current_state()

	#defining path to save trajevtory
 	rp = rospkg.RosPack()
        self._pkg_path = rp.get_path('pkg_task4')
        self._file_path = self._pkg_path + '/config/saved_trajectories/'
        rospy.loginfo( "Package Path: {}Fret".format(self._file_path) )

        
        
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
   	





	# Initialize Action Client
        self._ac = actionlib.ActionClient('/action_ros_iot',
                                          msgRosIotAction)
        
        # Dictionary to Store all the goal handels
        self._goal_handles = {}

        # Store the MQTT Topic on which to Publish in a variable
        param_config_iot = rospy.get_param('config_iot')
	self._config_mqtt_server_url = param_config_iot['mqtt']['server_url']
        self._config_mqtt_server_port = param_config_iot['mqtt']['server_port']
        self._config_mqtt_pub_topic = param_config_iot['mqtt']['topic_pub']#...................ros_to_iot
	self._config_mqtt_sub_topic = param_config_iot['mqtt']['topic_sub']#...................iot_to_ros
	self._config_mqtt_sub_cb_ros_topic = param_config_iot['mqtt']['sub_cb_ros_topic']#...............mqtt/sub
        # Wait for Action Server that will use the action - '/action_iot_ros' to start
        self._ac.wait_for_server()
	
	rospy.loginfo("to publish message from mqtt client to ros : eyrc/vaibhavs/iot_to_ros \n to receive data from ros mqtt client must subscribe to: eyrc/vaibhavs/ros_to_iot")

        rospy.loginfo("Action server up, we can send goals.")
	self._handle_ros_sub=rospy.Subscriber(self._config_mqtt_sub_cb_ros_topic,msgMqttSub,self.on_callback_from_bridge)

    # This function is called when there is a message from ros_iot bridge on the subscribed topic 
    def on_callback_from_bridge(self,data):
	rospy.loginfo("#############################################message received from bridge server by the action client" +str(data.orderID))
 	self._orders[data.orderID]=data.weight
   	self.orderID=sorted(self._orders, key=self._orders.get, reverse=True)
	rospy.loginfo(self._orders)
	
 
    # This function will be called when there is a change of state in the Action Client State Machine

    def on_transition(self, goal_handle):
        
        # from on_goal() to on_transition(). goal_handle generated by send_goal() is used here.
        
        result = msgRosIotResult()

        index = 0
        for i in self._goal_handles:
            if self._goal_handles[i] == goal_handle:
                index = i
                break

        rospy.loginfo("Transition Callback. Client Goal Handle #: " + str(index))
        rospy.loginfo("Comm. State: " + str(goal_handle.get_comm_state()) )
        rospy.loginfo("Goal Status: " + str(goal_handle.get_goal_status()) )
        
        # Comm State - Monitors the State Machine of the Client which is different from Server's
        # Comm State = 2 -> Active
        # Comm State = 3 -> Wating for Result
        # Comm State = 7 -> Done
        
        # if (Comm State == ACTIVE)
        if goal_handle.get_comm_state() == 2:
            rospy.loginfo(str(index) + ": Goal just went active.")
        
        # if (Comm State == DONE)
        if goal_handle.get_comm_state() == 7:
            rospy.loginfo(str(index) + ": Goal is DONE")
            rospy.loginfo(goal_handle.get_terminal_state())
            
            # get_result() gets the result produced by the Action Server
            result = goal_handle.get_result()
            rospy.loginfo(result.flag_success)

            if (result.flag_success == True):
                rospy.loginfo("Goal successfully completed. Client Goal Handle #: " + str(index))
            else:
                rospy.loginfo("Goal failed. Client Goal Handle #: " + str(index))





    # This function is used to send Goals to Action Server
    def send_goal_action(self, arg_protocol, arg_mode, arg_topic, arg_message):
        # Create a Goal Message object
        goal = msgRosIotGoal()

        goal.sheet1 = arg_protocol
        goal.order1 = arg_mode
        goal.sheet2= arg_topic
        goal.order2 = arg_message

        rospy.loginfo("Send goal.")
        
        # self.on_transition - It is a function pointer to a function which will be called when 
        #                       there is a change of state in the Action Client State Machine
        goal_handle = self._ac.send_goal(goal,
                                         self.on_transition,
                                         None)

        return goal_handle

    def get_dominant_colour(self, arg_img):
	    # setting values for base colors 
	    b = arg_img[:, :, 0] 
	    g = arg_img[:, :, 1] 
	    r = arg_img[:, :, 2] 
	  
	    # computing the mean 
	    b_mean = np.mean(b) 
	    g_mean = np.mean(g) 
	    r_mean = np.mean(r) 
	    #print(str(b_mean),str(g_mean),str(r_mean))
	    # displaying the most prominent color 
	    if (g_mean > r_mean and g_mean > b_mean): 
		#print("green")
		return 'GREEN' 
	    elif ((g_mean) < r_mean and r_mean > b_mean and g_mean != 91.15 ): 
		#print("red")
		return 'RED'
	    elif (b_mean > r_mean and g_mean < b_mean): 
		
		return 'BLUE'
	    else: 
		#print("yellow")
		return 'YELLOW'


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
  	    #cv2.imshow("/eyrc/vb/camera_1/image_raw",resized_image[310:390,120:240,:]
	    #cv2.imshow("/eyrc/vb/camera_1/image_raw", resized_image)
	    #rospy.loginfo(resized_image[310:390,120:240,:]))
	    for i in range(150,470,80):
    		for j in range(0,320,120):
			self.colours.append(self.get_dominant_colour(resized_image[i:(i+80),j:(j+120),:]))
			#rospy.loginfo(self.get_dominant_colour(resized_image[i:(i+80),j:(j+120):]))
 	    
	    #rospy.loginfo(self.colours)
	    cv2.waitKey(3)
	    self.image_sub.unregister()


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
    	#box_pose.pose.orientation.w= 1.0 
        box_pose.pose.position.z = 1.189#curr_pose.position.z
        box_pose.pose.position.x = -0.800#curr_pose.position.x
        box_pose.pose.position.y = 3.14#curr_pose.position.y-0.20
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
	    scene = self._scene

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
 	self._computed_plan = self._group.plan()
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
        self._group.stop()
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
 	self._group.stop()
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
    def automate(self,order,weight):
	orders=order
        weights=weight
        dict_color={1:"GREEN",
		    2:"YELLOW",
		    3:"RED"}
        if len(self._orders)>0:
		
   		for i in (self.pkg_clr):
			if self.pkg_clr[i] == dict_color[weight]:
   				del self.pkg_clr[i] 

				return i
	
    def ur5_1_automate_2(self,pkg):
        pkg_name=pkg
        file11='home_to_'+ pkg_name +'.yaml'
	file13=pkg_name +'_to_home.yaml'
	'''self.moveit_hard_play_planned_path_from_file()
	t11=Thread(target=self.moveit_hard_play_planned_path_from_file,args=(self._file_path, file11, 5))
	t13=Thread(target=self.moveit_hard_play_planned_path_from_file,args=(self._file_path, file13, 5))
	t11.start()'''
	self.moveit_hard_play_planned_path_from_file(self._file_path, file11, 5)
	self.gripper_ur5_1(True)
	self.moveit_hard_play_planned_path_from_file(self._file_path, file13, 5)
	self.gripper_ur5_1(False)
	self.conveyor(100)
	
    def ur5_1_automate_3(self,pkg):
        pkg_name1=pkg
        file11='home_to_'+pkg_name1+'.yaml'
        file12=pkg_name1 + '_break.yaml'
        file13=pkg_name1 + '_to_home.yaml'
        self.moveit_hard_play_planned_path_from_file(self._file_path, file11, 5)
	self.gripper_ur5_1(True)
	self.moveit_hard_play_planned_path_from_file(self._file_path, file12, 5)
	self.moveit_hard_play_planned_path_from_file(self.._file_path, file13, 5)
	self.gripper_ur5_1(False)
	self.conveyor(100)
	#sending sheet name 
    def ur5_2_automate(self)
        while(self.state==0):
                    continue
	    
        self.conveyor(0)
                
        self.gripper_ur5_2(True)
        file21=self.final_clr+'_to_bin.yaml'
        file22=self.final_clr+'_bin_to_home.yaml'
        self.moveit_hard_play_planned_path_from_file(self._file_path, file21, 5)
        self.gripper_ur5_2(False)
        #sending sheet name ,orderID,Dashboard,pkg color,
	#ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
        self.moveit_hard_play_planned_path_from_file(self._file_path, file22, 5)
			
    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
        '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')

def main():
    rospy.init_node("node_task_5")
    rospy.sleep(5)
    #gripper_ur5_1=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_1', vacuumGripper)
    #rospy.sleep(2)
    #gripper_ur5_2=rospy.ServiceProxy('/eyrc/vb/ur5/activate_vacuum_gripper/ur5_2', vacuumGripper)
    
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    print("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
    ur5_1=ur5_moveit("ur5_1")
    ur5_2=ur5_moveit("ur5_2")
    ur5_1.pkg_clr=dict(zip(ur5_1.packages,ur5_1.colours))
    ur5_2.pkg_clr=dict(zip(ur5_2.packages,ur5_2.colours))
    rospy.loginfo(sorted(ur5_1.pkg_clr.items(), key=lambda x: x[0], reverse=True))
    rospy.loginfo(ur5_1.colours)
    j=0
    
    for m in range(0,4):
	for n in range(0,3):
			color=ur5_1.colours[j]
			j+=1
                        storagenumber="R"+str(m)+" C"+str(n)
                        sku=color[0]+str(m)+str(n)+"0221"
			priority={
			"RED":"HP",
			"YELLOW":"MP",
			"GREEN":"LP"      
			 }
			cost={
			"RED":"450",
			"YELLOW":"250",
			"GREEN":"150"      
			 }
			item={
			"RED":"Medicine",
			"YELLOW":"Food",
			"GREEN":"Clothes"      
			 }
			parameters1 = {"id":"Inventory", "Team Id":"VB#0636", "Unique Id":"vaibhavs", "SKU":sku, "Item":item[color], "Priority":priority[color],"Storage Number":storagenumber,"Cost":cost[color]}

			#our sheeet
                        URL1 = "https://script.google.com/macros/s/AKfycbzvCRXARoeQXCFtm2qgThocqowU57BGCrBCBONe9fQLFS40GZrwbJg/exec"
			#eyantra sheet			
			#URL2 = "https://script.google.com/macros/s/AKfycbzlIlJ1EXloffSD2N3QdjiOVu8Qry042VjCrZecb05gtG3vK-h1/exec"
  
                    	#response_e1 = requests.get(URL2, params=parameters1)

                	response1 = requests.get(URL1, params=parameters1)
			#rospy.loginfo(self.get_dominant_colour(resized_image[i:(i+80),j:(j+120):]))
 	    		
    	
    #updating inventory sheet
    #ur5_1.send_goal_action("Inventory", str(ur5_1.pkg_clr),"Null","Null")
    del ur5_1.pkg_clr["pkg31"]
    del ur5_1.pkg_clr["pkg32"]
    del ur5_1.pkg_clr["pkg22"]
    del ur5_2.pkg_clr["pkg31"]
    del ur5_2.pkg_clr["pkg32"]
    del ur5_2.pkg_clr["pkg22"]		
	
    rospy.loginfo(ur5_1.pkg_clr)
    rospy.loginfo(len(ur5_1.pkg_clr))
    ur5_home_pose = geometry_msgs.msg.Pose()
    ur5_home_pose.position.x = -0.8
    ur5_home_pose.position.y = 0
    ur5_home_pose.position.z = 1.0+ vacuum_gripper_width + (box_length/2)

    # This to keep EE parallel to Ground Plane
    ur5_home_pose.orientation.x = -0.5
    ur5_home_pose.orientation.y = -0.5
    ur5_home_pose.orientation.z = 0.5
    ur5_home_pose.orientation.w = 0.5
    
    weights={
        1:"Clothes",
        2:"Food",
        3:"Medicine"}
    
    rospy.loginfo("sssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss")
    '''if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)
		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)
		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
    ####check for order###############################################################################################################################
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)
		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)
		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
    ####check for order###############################################################################################################################
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)
		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
    ####check for order###############################################################################################################################
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
    ####check for order###############################################################################################################################
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
    ####check for order###############################################################################################################################
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
    ####check for order###############################################################################################################################
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)
                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
    ####check for order###############################################################################################################################
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)

                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)

                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
    ####check for order###############################################################################################################################
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg_orderID=ur5_1.orderID[0]
	pkg_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
	    	
		file11='home_to_'+ pkg_name1 +'.yaml'
		file13=pkg_name1 +'_to_home.yaml'
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)

                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)
        else:
                file11='home_to_'+pkg_name1+'.yaml'
                file12=pkg_name1 + '_break.yaml'
                file13=pkg_name1 + '_to_home.yaml'
                ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
		gripper_ur5_1(True)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file12, 5)
		ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file13, 5)
		gripper_ur5_1(False)
		ur5_1.conveyor(100)

		#sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersDispatched",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
		##waiting for the package to arrive
		while(ur5_1.state==0):
                    continue
	    
                #gripper_ur5_1(True)
                ur5_1.conveyor(0)
                
                gripper_ur5_2(True)
                file21=ur5_2.final_clr+'_to_bin.yaml'
                file22=ur5_2.final_clr+'_bin_to_home.yaml'
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file21, 5)
                gripper_ur5_2(False)

                #sending sheet name ,orderID,Dashboard,pkg color,
		ur5_1.send_goal_action("OrdersShipped",pkg_orderID,weights,"Dashboard",weights[pkg_weight])
                ur5_2.moveit_hard_play_planned_path_from_file(ur5_2._file_path, file22, 5)'''
    
            
    ##########################################################ssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssssss
    if len(ur5_1._orders)==0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	pkg1_orderID=ur5_1.orderID[0]
	pkg1_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)

	del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
        if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
                    p1=Thread(target=ur5_1.ur5_1_automate_2,args=(pkg_name1, 5))
                    t1=Thread(target=ur5_2.ur5_2_automate,args=(5))
                    p1.start()
                    p1.join()
                    #sending sheet name ,orderID,Dashboard,pkg color,
                    ur5_1.send_goal_action("OrdersDispatched",pkg1_orderID,weights,"Dashboard",weights[pkg1_weight])
		
                    if len(ur5_1._orders)>0:
                        pkg_name2=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
                        pkg2_orderID=ur5_1.orderID[0]
                        pkg2_weight=ur5_1._orders[ur5_1.orderID[0]]
                        del ur5_1._orders[ur5_1.orderID[0]]
                        del ur5_2._orders[ur5_2.orderID[0]]

                        ur5_1.orderID.pop(0)
                        ur5_2.orderID.pop(0)
                        if (pkg_name2=="pkg00") or (pkg_name2=="pkg01") or (pkg_name2=="pkg02") or (pkg_name2=="pkg10") or (pkg_name2=="pkg11"):
                            p2=Thread(target=ur5_1.ur5_1_automate_2,args=(pkg_name1, 5))
                            t2=Thread(target=ur5_2.ur5_2_automate,args=(5))
                            t1.start()
                            p2.start()
                            t1.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersShipped",pkg1_orderID,weights,"Dashboard",weights[pkg1_weight])
                
                            p2.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersDispatched",pkg2_orderID,weights,"Dashboard",weights[pkg2_weight])
		
                            t2.start()
                            t2.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersShipped",pkg2_orderID,weights,"Dashboard",weights[pkg2_weight])
                
                        else :
                            p2=Thread(target=ur5_1.ur5_1_automate_3,args=(pkg_name1, 5))
                            t2=Thread(target=ur5_2.ur5_2_automate,args=(5))
                            t1.start()
                            p2.start()
                            t1.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersShipped",pkg1_orderID,weights,"Dashboard",weights[pkg1_weight])
                
                            p2.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersDispatched",pkg1_orderID,weights,"Dashboard",weights[pkg1_weight])
		
                            t2.start()
                            t2.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersShipped",pkg1_orderID,weights,"Dashboard",weights[pkg1_weight])
                
                    else:
                        t1.start()
                        t1.join()
                        #sending sheet name ,orderID,Dashboard,pkg color,
                        ur5_1.send_goal_action("OrdersShipped",pkg1_orderID,weights,"Dashboard",weights[pkg1_weight])
                
        else:
                    p1=Thread(target=ur5_1.ur5_1_automate_3,args=(pkg_name1, 5))
                    t1=Thread(target=ur5_2.ur5_2_automate,args=(5))
                    p1.start()
                    p1.join()

                    #sending sheet name ,orderID,Dashboard,pkg color,
                    ur5_1.send_goal_action("OrdersDispatched",pkg1_orderID,weights,"Dashboard",weights[pkg1_weight])
		
                    if len(ur5_1._orders)>0:
                        pkg_name2=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
                        p2_orderID=ur5_1.orderID[0]
                        p2_weight=ur5_1._orders[ur5_1.orderID[0]]
                        del ur5_1._orders[ur5_1.orderID[0]]
                        del ur5_2._orders[ur5_2.orderID[0]]

                        ur5_1.orderID.pop(0)
                        ur5_2.orderID.pop(0)
                        if (pkg_name2=="pkg00") or (pkg_name2=="pkg01") or (pkg_name2=="pkg02") or (pkg_name2=="pkg10") or (pkg_name2=="pkg11"):
                            p2=Thread(target=ur5_1.ur5_1_automate_2,args=(pkg_name1, 5))
                            t2=Thread(target=ur5_2.ur5_2_automate,args=(5))
                            t1.start()
                            p2.start()
                            t1.join()
                            p2.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersDispatched",pkg2_orderID,weights,"Dashboard",weights[pkg2_weight])
		
                            t2.start()
                        else :
                            p2=Thread(target=ur5_1.ur5_1_automate_3,args=(pkg_name1, 5))
                            t2=Thread(target=ur5_2.ur5_2_automate,args=(5))
                            t1.start()
                            p2.start()
                            t1.join()
                            p2.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersDispatched",pkg2_orderID,weights,"Dashboard",weights[pkg2_weight])
		
                            t2.start()
                            t2.join()
                            #sending sheet name ,orderID,Dashboard,pkg color,
                            ur5_1.send_goal_action("OrdersShipped",pkg2_orderID,weights,"Dashboard",weights[pkg2_weight])
                
                    else:
                        t1.start()
                        t1.join()
                        #sending sheet name ,orderID,Dashboard,pkg color,
                        ur5_1.send_goal_action("OrdersShipped",pkg1_orderID,weights,"Dashboard",weights[pkg1_weight])
                

	    
		                        

                    
        

    '''if len(ur5_1._orders)<=0: 
	while(len(ur5_1._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
	p1_orderID=ur5_1.orderID[0]
	p1_weight=ur5_1._orders[ur5_1.orderID[0]]
        del ur5_1._orders[ur5_1.orderID[0]]
	del ur5_2._orders[ur5_2.orderID[0]]

        del ur5_1.pkg_clr[pkg_name1]
        del ur5_2.pkg_clr[pkg_name1]
	ur5_1.orderID.pop(0)
	ur5_2.orderID.pop(0)
	if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
    	    file11='home_to_'+ pkg_name1 +'.yaml'
	    file13=pkg_name +'_to_home.yaml'
 	    ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, file11, 5)
            t11=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file11, 5))
	    t13=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file13, 5))
	    t11.start()
	    t11.join()
	    gripper_ur5_1(True)
	    t13.start()
	    t13.join()
     	    gripper_ur5_1(False)
            if len(ur5_1._orders)>0:
		pkg_name2=ur5_1.automate(ur5_1.orderID[0],ur5_1._orders[ur5_1.orderID[0]])
		p2_orderID=ur5_1.orderID[0]
		p2_weight=ur5_1._orders[ur5_1.orderID[0]]
		del ur5_1._orders[ur5_1.orderID[0]]
		del ur5_2._orders[ur5_2.orderID[0]]

		ur5_1.orderID.pop(0)
		ur5_2.orderID.pop(0)
		if (pkg_name1=="pkg00") or (pkg_name1=="pkg01") or (pkg_name1=="pkg02") or (pkg_name1=="pkg10") or (pkg_name1=="pkg11"):
		        #gripper_ur5_1(True)
			c
			
	    ur5_1.conveyor(100)
	    
	    
   	    file11='home_to_'+ pkg_name1 +'.yaml'
	    file13=pkg_name +'_to_home.yaml'
	    t11=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file11, 5))
	    t13=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file13, 5))
	    t11.start()
	    t11.join()
	    gripper_ur5_1(True)
	    t13.start()
	    
	    
	    
	    gripper_ur5_2(True)
	    file21=ur5_2.final_clr+'_to_bin.yaml'
	    file22=ur5_2.final_clr+'_bin_to_home.yaml'
	    t21=Thread(target=ur5_2.moveit_hard_play_planned_path_from_file,args=(ur5_2._file_path,file21, 5))
	    t22=Thread(target=ur5_2.moveit_hard_play_planned_path_from_file,args=(ur5_2._file_path,file22, 5))
	    #t12.start()
	    gripper_ur5_2(True)
	    t21.start()
	    
	    t13.join()
	    gripper_ur5_1(False)
	    
	    t21.join()
	    gripper_ur5_2(False)
	    t22.start()
	    
	    gripper_ur5_1(False)
	    ur5_1.conveyor(100)    

    else:

	    
	    file11='home_to_'+pkg_name1+'.yaml'
	    file12=pkg_name1 + '_break.yaml'
	    file13=pkg_name1 + '_to_home.yaml'
	    t11=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file11, 5))
	    t12=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file12, 5))
	    t13=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file13, 5))
	    t11.start()
	    t11.join()
	    gripper_ur5_1(True)
	    t12.start()
	    
	    while(ur5_1.state==0):
		#ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'home_to_pkg12.yaml', 5)
		continue
	    
	    #gripper_ur5_1(True)
	    ur5_1.conveyor(0)
	    
	    gripper_ur5_2(True)
	    file21=ur5_2.final_clr+'_to_bin.yaml'
	    file22=ur5_2.final_clr+'_bin_to_home.yaml'
	    t21=Thread(target=ur5_2.moveit_hard_play_planned_path_from_file,args=(ur5_2._file_path,file21, 5))
	    t22=Thread(target=ur5_2.moveit_hard_play_planned_path_from_file,args=(ur5_2._file_path,file22, 5))
	    #t12.start()
	    gripper_ur5_2(True)
	    t21.start()
	    t12.join()
	    t13.start()
	    t21.join()
	    gripper_ur5_2(False)
	    t13.join()
	    gripper_ur5_1(False)
	    t22.start()
	    
	    ur5_1.conveyor(100)
	   
    ###################### 
    if len(self._orders)<=0: 
	while(len(self._orders)<=0):
		continue
    else:
	pkg_name1=ur5_1.automate(self.orderID[0],self._orders[self.orderID[0]])
	if (pkg_name=="pkg00") or (pkg_name=="pkg01") or (pkg_name=="pkg02") or (pkg_name=="pkg10") or (pkg_name=="pkg11"):
	    file11='home_to_'+ pkg_name +'.yaml'
	    file13=pkg_name +'_to_home.yaml'
	    ur5_1.moveit_hard_play_planned_path_from_file()
	    t11=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file11, 5))
	    t13=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file13, 5))
	    t11.start()
	    t11.join()
	    gripper_ur5_1(True)
	    t13.start()
	    
	    while(ur5_1.state==0):
		continue
	    
	    #gripper_ur5_1(True)
	    ur5_1.conveyor(0)
	    
	    gripper_ur5_2(True)
	    file21=ur5_2.final_clr+'_to_bin.yaml'
	    file22=ur5_2.final_clr+'_bin_to_home.yaml'
	    t21=Thread(target=ur5_2.moveit_hard_play_planned_path_from_file,args=(ur5_2._file_path,file21, 5))
	    t22=Thread(target=ur5_2.moveit_hard_play_planned_path_from_file,args=(ur5_2._file_path,file22, 5))
	    #t12.start()
	    gripper_ur5_2(True)
	    t21.start()
	    
	    t13.join()
	    gripper_ur5_1(False)
	    
	    t21.join()
	    gripper_ur5_2(False)
	    t22.start()
	    
	    gripper_ur5_1(False)
	    ur5_1.conveyor(100)    

    else:

	    
	    file11='home_to_'+pkg12+'.yaml'
	    file12='pkg12_break.yaml'
	    file13='pkg12_to_home.yaml'
	    t11=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file11, 5))
	    t12=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file12, 5))
	    t13=Thread(target=ur5_1.moveit_hard_play_planned_path_from_file,args=(ur5_1._file_path, file13, 5))
	    t11.start()
	    t11.join()
	    gripper_ur5_1(True)
	    t12.start()
	    
	    while(ur5_1.state==0):
		#ur5_1.moveit_hard_play_planned_path_from_file(ur5_1._file_path, 'home_to_pkg12.yaml', 5)
		continue
	    
	    #gripper_ur5_1(True)
	    ur5_1.conveyor(0)
	    
	    gripper_ur5_2(True)
	    file21=ur5_2.final_clr+'_to_bin.yaml'
	    file22=ur5_2.final_clr+'_bin_to_home.yaml'
	    t21=Thread(target=ur5_2.moveit_hard_play_planned_path_from_file,args=(ur5_2._file_path,file21, 5))
	    t22=Thread(target=ur5_2.moveit_hard_play_planned_path_from_file,args=(ur5_2._file_path,file22, 5))
	    #t12.start()
	    gripper_ur5_2(True)
	    t21.start()
	    t12.join()
	    t13.start()
	    t21.join()
	    gripper_ur5_2(False)
	    t13.join()
	    gripper_ur5_1(False)
	    t22.start()
	    
	    ur5_1.conveyor(100)'''
    
    
   

if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException:
        pass
