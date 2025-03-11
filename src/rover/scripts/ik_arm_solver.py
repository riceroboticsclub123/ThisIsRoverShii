import PyKDL as kdl
import math

# Define the same 3-joint chain as in your forward kinematics example
chain = kdl.Chain()

# Joint 1
joint1 = kdl.Joint(kdl.Joint.RotZ)  # Revolute joint around Z-axis
frame1 = kdl.Segment(joint1, kdl.Frame(kdl.Vector(0.5, 0, 0)))
chain.addSegment(frame1)

# Joint 2
joint2 = kdl.Joint(kdl.Joint.RotZ)
frame2 = kdl.Segment(joint2, kdl.Frame(kdl.Vector(0.5, 0, 0)))
chain.addSegment(frame2)

# Joint 3
joint3 = kdl.Joint(kdl.Joint.RotZ)
frame3 = kdl.Segment(joint3, kdl.Frame(kdl.Vector(0.5, 0, 0)))
chain.addSegment(frame3)

# Create forward kinematics solver
fk_solver = kdl.ChainFkSolverPos_recursive(chain)

# Create inverse kinematics solver
ik_solver = kdl.ChainIkSolverPos_LMA(chain)

# Define the desired end-effector pose (target)
target_pose = kdl.Frame(kdl.Rotation.RPY(0.0, 0.0, 0.0), kdl.Vector(1.0, 1.0, 0.0))  # Example target position

# Calculate the sum of link lengths
link_length = 0.5
max_reachable_distance = 3 * link_length  # Since we have 3 links of length 0.5 each

# Check if the target is within the reachable distance
target_distance = math.sqrt(target_pose.p.x()**2 + target_pose.p.y()**2)

if target_distance > max_reachable_distance:
    print("Target is out of reachable workspace.")
else:
    print("Target is within reachable workspace.")

    # Create an array for joint positions
    num_joints = chain.getNrOfJoints()
    joint_positions = kdl.JntArray(num_joints)

    # Initialize joint positions to some guess (optional, but necessary for the solver)
    joint_positions[0] = 0.0
    joint_positions[1] = 0.0
    joint_positions[2] = 0.0

    # Create an array to store the solution (output joint angles)
    solution = kdl.JntArray(num_joints)

    # Compute inverse kinematics to reach the target pose
    ik_result = ik_solver.CartToJnt(joint_positions, target_pose, solution)  # Solve IK

    # Check if IK solution was successful
    if ik_result >= 0:
        print("Inverse Kinematics solution found:")
        print(f"Joint angles (radians): {solution[0]}, {solution[1]}, {solution[2]}")
    else:
        print("Inverse Kinematics solution failed")

    # To verify, compute forward kinematics using the obtained joint angles
    end_effector_pose = kdl.Frame()
    fk_solver.JntToCart(solution, end_effector_pose)

    # Print result to see if the end effector reaches the target
    print(f"End-Effector Position: {end_effector_pose.p.x()}, {end_effector_pose.p.y()}, {end_effector_pose.p.z()}")