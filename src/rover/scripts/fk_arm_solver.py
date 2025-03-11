#from sensor_msgs.msg import JointState
#from geometry_msgs.msg import Twist
import PyKDL as kdl
#import a parser

# Define a simple 3-joint chain
# counterclockwise for theta is psoitive
# angles are compared to last linkaged (in line would be 0 degrees)
# first join angle arbitrary

chain = kdl.Chain()
joint1 = kdl.Joint(kdl.Joint.RotZ)  # Revolute joint around Z-axis
frame1 = kdl.Segment(joint1, kdl.Frame(kdl.Vector(0.5, 0, 0)))
chain.addSegment(frame1)

joint2 = kdl.Joint(kdl.Joint.RotZ)
frame2 = kdl.Segment(joint2, kdl.Frame(kdl.Vector(0.5, 0, 0)))
chain.addSegment(frame2)

joint3 = kdl.Joint(kdl.Joint.RotZ)
frame3 = kdl.Segment(joint3, kdl.Frame(kdl.Vector(0.5, 0, 0)))
chain.addSegment(frame3)

# Create a KDL solver
fk_solver = kdl.ChainFkSolverPos_recursive(chain)

# Set joint positions
num_joints = chain.getNrOfJoints()
joint_positions = kdl.JntArray(num_joints)
joint_positions[0] = 0.5  # Example joint angles (radians)
joint_positions[1] = 1.0
joint_positions[2] = -0.5

# Compute forward kinematics
end_effector_pose = kdl.Frame()
fk_solver.JntToCart(joint_positions, end_effector_pose)

# Print result
print(f"End-Effector Position: {end_effector_pose.p.x()}, {end_effector_pose.p.y()}, {end_effector_pose.p.z()}")