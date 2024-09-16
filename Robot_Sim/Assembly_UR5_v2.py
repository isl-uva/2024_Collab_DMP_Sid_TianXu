import pybullet as p
import pybullet_data
import time
import yaml 
import pdb
import numpy as np
from scipy.spatial.transform import rotation as R
from utilities import euler_to_quaternion
import csv
import itertools

from robots import ur5_simple, ur5_robotiq, panda, kuka


def create_3d_grid_world(objs, block_size, obs, block_mass, offset): 

    ###############################################
    # Setup Blocks
    ###############################################

    for i,k in enumerate(list(objs.keys())):      
        # Create a box at the specified position with the specified color

        box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")
        position_ = objs[k][0]

        position = [x*block_size + block_size/2 + offset[i] for i,x in enumerate(position_)]

        position = tuple(position)
        print(position)

        goal_position_ = objs[k][-1]
        goal_position = []
        goal_position = [x*block_size + block_size/2 + offset[i] for i,x in enumerate(goal_position_)]
        goal_position = tuple(goal_position)
        
        p.createMultiBody(baseMass=block_mass, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        
        startOrientation = [0,0,0,1]
        p.resetBasePositionAndOrientation(box_id, position, startOrientation)
        objs[k] = [position,box_id] + objs[k] + [goal_position]
    


    ###############################################
    # Setup Obstacles
    ###############################################

    for i,ob in enumerate(obs):      
        # Create a box at the specified position with the specified color
        box_id = p.loadURDF("Robot_Sim/urdf/object/box/obs.urdf")
        position_ = ob
        position = [x*block_size + block_size/2 + offset[i] for i,x in enumerate(position_)]

        position = tuple(position)

        color = [0,0,0]
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        p.changeVisualShape(box_id, -1, rgbaColor=color)


def init_setup_robot():
    # Connect to the PyBullet physics server
    p.connect(p.GUI)
    # p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    
    # # Set gravity
    p.setGravity(0, 0, -9.81)
    
    # # Load the plane as the ground
    plane_id = p.loadURDF("plane.urdf")

    # robot_urdf = "Robot_Sim/urdf/ur5/ur5e.urdf"
    # robot_urdf = "Robot_Sim/urdf/ur5/ur5.urdf"
    # cubeStartPos = [0,0,0]
    # cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    # # robot_urdf = "/home/Siddharth/Siddharth/PyBullet/urdf/ur5/ur5_simple_gripper.urdf"
    # robot_id = p.loadURDF(robot_urdf, cubeStartPos, cubeStartOrientation,useFixedBase=True)
    # start_pose = [0., 0., -2.137, 1.432, -0.915, -1.591, 0.071]
    # num_joints = 7
    # [p.resetJointState(robot_id, idx, start_pose[idx]) for idx in range(num_joints)]

    # return robot_id

# def update_block_state(robot_id, objs):
#     x_ee = p.getLinkState(robot_id,8)
    
#     for j,k in enumerate(list(objs.keys())):
#         demo_pos = objs[k][0]

#         demo_ori = p.getQuaternionFromEuler([0,0,0])

#         UL = [3.14,3.14,3.14,3.14,3.14,3.14]
#         LL = [-3.14,-3.14,-3.14,-3.14,-3.14,-3.14]

#         joint_angles = p.calculateInverseKinematics(robot_id,7,demo_pos,demo_ori,LL,UL,maxNumIterations=20)
#         pdb.set_trace()
#         num_joints = 7
#         [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(num_joints)]
#         x_ee = p.getLinkState(robot_id,8)
#         print("End effector is at: ", x_ee)
    




    
if __name__=="__main__":
    from types import SimpleNamespace as SN
    import yaml
    with open('Robot_Sim/Block_stacking_DQN.yaml', 'r') as f:
        config = SN(**yaml.load(f, Loader=yaml.FullLoader))
    print(config)

    min_p = tuple(config.env['min_p'])
    max_p = tuple(config.env['max_p'])
    obs = []
    for ob in config.env['obs']:
        obs.append(tuple(ob))

    objs = config.env['objs']
    for k in list(objs.keys()):
        objs[k][0] = tuple(objs[k][0])

    block_size = config.pybullet["block_size"]
    block_mass = config.pybullet["block_mass"]
    offset = config.pybullet["offset"]

    # robot_id = init_setup_robot()
    init_setup_robot()
    robot_urdf = "Robot_Sim/urdf/ur5/ur5_robotiq_85_gripper_fake.urdf"
    cubeStartPos = [0,0,0]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    robot_id = p.loadURDF(robot_urdf, cubeStartPos, cubeStartOrientation,useFixedBase=True)
    start_pose = [0., 0., -2.137, 1.432, -0.915, -1.591, 0.071, 0]
    num_joints = 8
    [p.resetJointState(robot_id, idx, start_pose[idx]) for idx in range(num_joints)]
    for i in range (num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(joint_info)


    create_3d_grid_world(objs, block_size, obs, block_mass, offset)
    
    ee_index = 12
    

    reset = True
    while reset is True:
        i = 0
        file_path_pos = open("Robot_Sim/pick_place_demo_pos.csv","r")
        file_path_ori = open("Robot_Sim/pick_place_demo_ori.csv","r")
        demo_pos_reader = csv.reader(file_path_pos)
        demo_ori_reader = csv.reader(file_path_ori)

        for demo_pos,demo_ori in itertools.zip_longest(demo_pos_reader,demo_ori_reader):
            i += 1
            print("<<----------oo",i,"oo--------->>")
            
            demo_pos = [float(pos) for pos in demo_pos[:-1]]
            demo_ori = [float(ori) for ori in demo_ori[:-1]]
            print(demo_pos)
            print(demo_ori)
            
            joint_angles = p.calculateInverseKinematics(robot_id,ee_index,demo_pos, demo_ori,maxNumIterations=100)
            joint_angles = [0] + list(joint_angles)
            num_joints = len(joint_angles)
            [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
            x_ee = p.getLinkState(robot_id,ee_index)[0]
            print("End effector is at: ", x_ee)
            time.sleep(1)
        reset = True
        

        