# Import the necessary Python modules # rospy - ROS Python API
import rospy # intera_interface - Sawyer Python API
import intera_interface # initialize our ROS node, registering it with the Master 

rospy.init_node('Hello_Sawyer') # create an instance of intera_interface's Limb class
limb = intera_interface.Limb('right') # get the right limb's current joint angles
#angles = limb.joint_angles() # print the current joint angles
#print(angles) # move to neutral pose
limb.move_to_neutral() # get the right limb's current joint angles now that it is in neutral

angles = limb.joint_angles() # print the current joint angles again
for joint_name, angle in angles.items():
    print(joint_name, angle)
