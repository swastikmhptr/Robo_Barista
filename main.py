import sys
sys.path.append("/home/ros_ws")
from frankapy.franka_arm import FrankaArm
from src.devel_packages.manipulation.src.moveit_class import MoveItPlanner
from geometry_msgs.msg import Pose
from moveit_commander.move_group import MoveGroupCommander
import moveit_commander
from std_msgs.msg import Header
from map_scene import add_objs_to_scene
from time import sleep
from moveit_msgs.msg import Constraints, OrientationConstraint
from math import radians
from WeighingScale import WeighingScale
import numpy as np
from localize_output_cup import get_output_cup_location

franka_moveit = MoveItPlanner()
franka_moveit.group.set_planner_id("RRTStar")
scene = moveit_commander.PlanningSceneInterface()
scene.clear()
franka_moveit.group.clear_path_constraints()
franka_moveit.group.set_goal_tolerance(0.001)  # Applies to position, orientation, and joint tolerances
add_objs_to_scene(scene)
exit(0)
franka_moveit.fa.open_gripper()

# HOME_POSE = RigidTransform(rotation=np.array([
#         [1, 0, 0],
#         [0, -1, 0],
#         [0, 0, -1],
#     ]), translation=np.array([0.3069, 0, 0.4867]),
#     from_frame='franka_tool', to_frame='world')

def plan_and_go(xyz, wxyz, mode):
    pose_goal = Pose()
    pose_goal.position.x = xyz[0]
    pose_goal.position.y = xyz[1]
    pose_goal.position.z = xyz[2]
    pose_goal.orientation.w = wxyz[0]
    pose_goal.orientation.x = wxyz[1]
    pose_goal.orientation.y = wxyz[2]
    pose_goal.orientation.z = wxyz[3]
    pose_goal_moveit = franka_moveit.get_moveit_pose_given_frankapy_pose(pose_goal)
    if mode == 'straight':
        plan = franka_moveit.get_straight_plan_given_pose(pose_goal_moveit)
    elif mode == 'plan':
        plan = franka_moveit.get_plan_given_pose(pose_goal_moveit)
    franka_moveit.execute_plan(plan)
    sleep(1)

def get_offset_angle(weight):
    lookup_table = {
        (0,30): (0, -1.0),
        (30,50): (6, -0.88),
        (50,80): (6, -0.88),
        (80,150): (10, -0.875),
    }
    for it in lookup_table.items():
        print(it)
    for (low, high), (offset, angle) in lookup_table.items():
        if low <= weight < high:
            print(low, high, weight, offset, angle)
            return offset, angle
    raise ValueError(f"No mapping found for weight {weight}")

def Pour(in_cup_wt, target_weight):
# using a time-based discretized pouring loop 
    

    previous_weight = scale.read_weight()
    # stopping_offset = 4
    # pour_angle = -0.86

    stopping_offset, pour_angle = get_offset_angle(in_cup_wt)
    print(f"Offset: {stopping_offset}, Pour angle: {pour_angle}")

    current_weight = scale.read_weight()
    print(f"Initial weight: {current_weight} g")

    joints = franka_moveit.fa.get_joints()
    pour_joints = joints 

    pour_joints[-1] =  pour_joints[-1] + pour_angle
    poured = False
    franka_moveit.fa.goto_joints(pour_joints, ignore_virtual_walls=True)

    while(1):
        print("pouring")
        current_weight = scale.read_weight()
        print(f"Current weight: {current_weight} g")
        if current_weight is None:
            continue
        if current_weight > target_weight - stopping_offset:  
            poured = True
            break

    if poured == True:
        joints = franka_moveit.fa.get_joints()
        joints[-1] = joints[-1] + np.pi/2
        franka_moveit.fa.goto_joints(joints, ignore_virtual_walls=True)

        final_weight = scale.read_weight()
        print(f"Final weight poured: {final_weight - previous_weight} g")
        
# franka_moveit.fa.reset_joints()
franka_moveit.reset_joints()
sleep(1)
## Cam Pose at Vantage Point
# Tra: [0.50931026 0.12884678 0.64182019]
#  Rot: [[-0.99685539  0.03060031 -0.07296432]
#  [ 0.03328591  0.998792   -0.03587969]
#  [ 0.07177825 -0.03819555 -0.99668895]]
#  Qtn: [-0.03622036  0.01598688  0.99904158 -0.01853658]
#  from franka_tool to world


# [ 0.2163256   0.16941512  0.04844499 -0.99417548 -0.02684299  1.08350032
#  -2.07130746]
franka_moveit.fa.goto_joints([0.2163256, 0.16941512, 0.04844499, -0.99417548, -0.02684299, 1.08350032, -2.07130746])
ret = get_output_cup_location()
cup_pose_loc = ret[0]
cup_pose_loc[0] = cup_pose_loc[0] - -0.01341144
cup_pose_loc[1] = cup_pose_loc[1] - 0.02141274

print(cup_pose_loc)
# exit(0)
# sleep(5)

# # # final_cup_pose = get_cup_pose()
# final_cup_pose = [0.50214494, 0.14494402, 0.35]
final_cup_pose = [0.45, 0.14494402, 0.35]


final_cup_pose[0] -= cup_pose_loc[1]
final_cup_pose[1] -= cup_pose_loc[0]

franka_moveit.reset_joints()

# sleep(5)
plan_and_go([0.3069, 0, 0.4867], [ 0, 1, 0, 0], 'plan')
sleep(2)
plan_and_go([0.40,  0.20, 0.2026981], [ 0.50136373,  0.51176609, -0.46587643,  0.51930632], 'plan')

scene.remove_world_object("cup1")

plan_and_go([0.40,  0.15, 0.2026981], [ 0.50136373,  0.51176609, -0.46587643,  0.51930632], 'plan')
franka_moveit.fa.close_gripper()

plan_and_go(final_cup_pose, [ 0.63393339,  0.65902103, -0.26453478,  0.30632524], 'plan')
# exit(0)
scale = WeighingScale()
current_wt = scale.read_weight()
while current_wt is None:
    current_wt = scale.read_weight()

force = franka_moveit.fa.get_ee_force_torque()
print(force)
input_cup_weight = ((force[2] / 9.81)  - 0.135877 - 0.055) * 1000
print(f"Input cup weight: {input_cup_weight} g")
pre_pour_joints = franka_moveit.fa.get_joints()

Pour(input_cup_weight, current_wt + 60)

plan_and_go([0.40,  0.15, 0.2026981], [ 0.50136373,  0.51176609, -0.46587643,  0.51930632], 'plan')

franka_moveit.fa.open_gripper()

plan_and_go([0.40,  0.20, 0.2026981], [ 0.50136373,  0.51176609, -0.46587643,  0.51930632], 'plan')

franka_moveit.fa.reset_joints()