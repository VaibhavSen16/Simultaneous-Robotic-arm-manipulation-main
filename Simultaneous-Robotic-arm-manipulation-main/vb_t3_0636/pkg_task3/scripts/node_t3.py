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

class CartesianPath:

    # Constructor
    def __init__(self):
	
        self.state=0
	#self.gripper=rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
	self.logical_camera=rospy.Subscriber("/eyrc/vb/logical_camera_2",LogicalCameraImage, self.Logical_camera_callback,queue_size=1)
	self.conveyor = rospy.ServiceProxy('/eyrc/vb/conveyor/set_power', conveyorBeltPowerMsg)#to activate conveyer belt
        self._planning_group = "ur5_1_planning_group"
        self._commander = moveit_commander.roscpp_initialize(sys.argv)
        self._robot = moveit_commander.RobotCommander()
        self._scene = moveit_commander.PlanningSceneInterface()
        self._group = moveit_commander.MoveGroupCommander(self._planning_group)
        self._display_trajectory_publisher = rospy.Publisher(
            '/move_group/display_planned_path', moveit_msgs.msg.DisplayTrajectory, queue_size=1)
	
        #self._exectute_trajectory_client = actionlib.SimpleActionClient('execute_trajectory', moveit_msgs.msg.ExecuteTrajectoryAction)
        #self._exectute_trajectory_client.wait_for_server()

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

    def Logical_camera_callback(self,msg):
	print(len(msg.models))
        #print(msg.models[0].type)
        print(msg)
	#print(msg.pose.position.x,msg.pose.position.y,msg.pose.position.z)
    	if len(msg.models) > 0 :
		if msg.models[-1].type in ["packagen1","packagen2","packagen3"]:

			if round(msg.models[-1].pose.position.y,1)==0.0:
				

				self.state=1
				print(self.state)
				
  	else:
		self.state=0
   				

    def ee_cartesian_translation(self, trans_x, trans_y, trans_z):
        # 1. Create a empty list to hold waypoints
        waypoints = []
	self.start=0
        # 2. Add Current Pose to the list of waypoints
        waypoints.append(self._group.get_current_pose().pose)

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

    
    def go_to_pose(self, arg_pose):

        pose_values = self._group.get_current_pose().pose
        rospy.loginfo('\033[94m' + ">>> Current Pose:" + '\033[0m')
        rospy.loginfo(pose_values)
	self.state=0
        self._group.set_pose_target(arg_pose)
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


    # Destructor
    def __del__(self):
        moveit_commander.roscpp_shutdown()
        rospy.loginfo(
        '\033[94m' + "Object of class CartesianPath Deleted." + '\033[0m')
###################################################################################


class tfEcho(CartesianPath):

    def __init__(self):
        #rospy.init_node('node_tf_echo')
	CartesianPath.__init__(self)
        self._tfBuffer = tf2_ros.Buffer()
        self._listener = tf2_ros.TransformListener(self._tfBuffer)
	self._pose=geometry_msgs.msg.Pose()

    def func_tf_print(self, arg_frame_1, arg_frame_2):
        try:
            trans = self._tfBuffer.lookup_transform(arg_frame_1, arg_frame_2, rospy.Time())
	    self.state=0
            rospy.loginfo(  "\n" +
                            "Translation: \n" +
                            "x: {} \n".format(trans.transform.translation.x) +
                            "y: {} \n".format(trans.transform.translation.y) +
                            "z: {} \n".format(trans.transform.translation.z) +
                            "\n" +
                            "Orientation: \n" +
                            "x: {} \n".format(trans.transform.rotation.x) +
                            "y: {} \n".format(trans.transform.rotation.y) +
                            "z: {} \n".format(trans.transform.rotation.z) +
                            "w: {} \n".format(trans.transform.rotation.w) )
	    self._pose.position.x= (-1)*trans.transform.translation.z
	    self._pose.position.y=trans.transform.translation.x
	    self._pose.position.z= -1*(trans.transform.translation.y - 0.197)
	    self._pose.orientation.x=trans.transform.rotation.x
	    self._pose.orientation.y=trans.transform.rotation.y
	    self._pose.orientation.z=trans.transform.rotation.z
	    self._pose.orientation.w=trans.transform.rotation.w
            self.ee_cartesian_translation(self._pose.position.x,self._pose.position.y,self._pose.position.z)

	    return self._pose

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            rospy.logerr("TF error")

################################################################################


	

def main():
    rospy.init_node("node task 3")
    
   
    ur5_pose_1 = geometry_msgs.msg.Pose()
    ur5_pose_1.position.x = 0.598978209204
    ur5_pose_1.position.y = 0.213119414841
    ur5_pose_1.position.z = 1.13093406061
    ur5_pose_1.orientation.x = -0.000263851030942
    ur5_pose_1.orientation.y = 0.705635523536
    ur5_pose_1.orientation.z =-0.708574925692
    ur5_pose_1.orientation.w = 0.000113962676622
    


    ur5_pose_2 = geometry_msgs.msg.Pose()
    ur5_pose_2.position.x =-0.101329840758
    ur5_pose_2.position.y =-0.545843469289
    ur5_pose_2.position.z = 1.32248663163
    ur5_pose_2.orientation.x = -0.499664561444
    ur5_pose_2.orientation.y = -0.499953712369
    ur5_pose_2.orientation.z = 0.499851129103
    ur5_pose_2.orientation.w =0.50053017917



    ur5_pose_3 = geometry_msgs.msg.Pose()
    ur5_pose_3.position.x = 0.0342009771042
    ur5_pose_3.position.y = 0.700428334843
    ur5_pose_3.position.z = 1.24463137182
    ur5_pose_3.orientation.x = -0.500323940784
    ur5_pose_3.orientation.y = -0.500121038077
    ur5_pose_3.orientation.z =0.500148199501
    ur5_pose_3.orientation.w = 0.49940632764
    ur5 = CartesianPath()  
    my_tf = tfEcho() 
    frame1 = "world"
    frame2 = "ur5_wrist_3_link"
    frame3 = "logical_camera_2_frame"
    frame4 = "logical_camera_2_packagen1_frame"
    frame5 = "logical_camera_2_packagen2_frame"
    frame6 = "logical_camera_2_packagen3_frame"
    frame7= "ur5_ee_link"
    frame8="ur5_1_frame"
    frame9="ur5_1_packagen2_frame"
    box_length = 0.15               # Length of the Package
    vacuum_gripper_width = 0.115    # Vacuum Gripper Width
    delta = vacuum_gripper_width + (box_length/2)  # 0.19
    
    gripper=rospy.ServiceProxy('/eyrc/vb/ur5_1/activate_vacuum_gripper', vacuumGripper)
    rospy.sleep(3)
    
    #rospy.sleep(3)
    
    ur5_2_home_pose = geometry_msgs.msg.Pose()
    ur5_2_home_pose.position.x = -0.8
    ur5_2_home_pose.position.y = 0
    ur5_2_home_pose.position.z = 1.0+ vacuum_gripper_width + (box_length/2)
    # This to keep EE parallel to Ground Plane
    ur5_2_home_pose.orientation.x = -0.5
    ur5_2_home_pose.orientation.y = -0.5
    ur5_2_home_pose.orientation.z = 0.5
    ur5_2_home_pose.orientation.w = 0.5
    ur5.go_to_pose(ur5_2_home_pose)
    ur5.conveyor(100)
    #my_tf.func_tf_print(reference_frame, target_frame)
 
    #waiting for first package to reach location
    while(ur5.state==0):

	continue    
    ur5.conveyor(0)    
    gripper(True)
    ur5.ee_cartesian_translation(0,0,0.05) # to pick the package to a certain height and then movement will occur of the arm preventing the collision
    ur5.go_to_pose(ur5_pose_3)
    gripper(False)
    ur5.go_to_pose(ur5_2_home_pose)
    ur5.conveyor(100)



    #waiting for second package to reach location
    while(ur5.state==0):

        continue
    ur5.conveyor(0)
    ur5.go_to_pose(ur5_2_home_pose)
    my_tf.func_tf_print(frame2,frame5)
    gripper(True)
    ur5.ee_cartesian_translation(0,0,0.05)   # to pick the package to a certain height and then movement will occur of the arm preventing the collision
    ur5.go_to_pose(ur5_pose_1)
    gripper(False)
    ur5.conveyor(100)

    #waiting for third package to reach location
    while(ur5.state==0):

        continue
    
    ur5.conveyor(0)   
    ur5.go_to_pose(ur5_2_home_pose)
    my_tf.func_tf_print(frame2,frame6)
    gripper(True)
    ur5.ee_cartesian_translation(0,0,0.1)   # to pick the package to a certain height and then movement will occur of the arm preventing the collision
    ur5.go_to_pose(ur5_pose_2)
    gripper(False)
    ur5.go_to_pose(ur5_2_home_pose)   

    sys.exit()
  

    

    
    
    


if __name__ == '__main__':
    try:
	main()
    except rospy.ROSInterruptException:
        pass
