
# Import the necessary Python modules # rospy - ROS Python API
import rospy # intera_interface - Sawyer Python API
import intera_interface # initialize our ROS node, registering it with the Master 

rospy.init_node('Stretch_Arm') # create an instance of intera_interface's Limb class
limb = intera_interface.Limb('right') # get the right limb's current joint angles
angles = limb.joint_angles() # print the current joint angles

angles['right_j0']=0.0
angles['right_j1']=0.0 
angles['right_j2']=0.0
angles['right_j3']=0.0
angles['right_j4']=0.0
angles['right_j5']=0.0
angles['right_j6']=0.0

for joint_name, angle in angles.items():
    print(joint_name, angle)

limb.move_to_joint_positions(angles) # Sawyer wants t stretch
