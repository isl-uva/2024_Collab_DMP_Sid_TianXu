import pybullet as p
import pybullet_data
import time
import yaml 
import pdb
import numpy as np
from scipy.spatial.transform import rotation as R
from utilities import euler_to_quaternion
import csv, json
from ast import literal_eval

from load_from_excel import *

from robots import ur5_simple, ur5_robotiq, panda, kuka


def create_3d_grid_world(objs, block_size, obs, block_mass, offset): 

    ###############################################
    # Setup Blocks
    ###############################################

    for i,k in enumerate(list(objs.keys())):      
        # Create a box at the specified position with the specified color

        box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")
        position = objs[k][1]

        # position = [x*block_size + block_size/2 + offset[i] for i,x in enumerate(position_)]

        position = tuple(position)
        print(position)

        goal_position_ = objs[k][3]
        goal_position = []
        goal_position = [x*block_size + block_size/2 + offset[i] for i,x in enumerate(goal_position_)]
        goal_position = tuple(goal_position)
        
        p.createMultiBody(baseMass=block_mass, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        
        startOrientation = [0,0,0,1]
        p.resetBasePositionAndOrientation(box_id, position, startOrientation)
        objs[k] = [box_id] + objs[k]
    


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
    # p.connect(p.DIRECt)

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
    with open('HL_LfD_Combine/Block_stacking_HL.yaml', 'r') as f:
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
    # robot_urdf = "Robot_Sim/urdf/ur5/ur5_robotiq_85_gripper_fake.urdf"
    robot_urdf = "Robot_Sim/urdf/ur5/ur5.urdf"
    cubeStartPos = [0,0,0]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    robot_id = p.loadURDF(robot_urdf, cubeStartPos, cubeStartOrientation,useFixedBase=True)
    start_pose = [0., 0., -2.137, 1.432, -0.915, -1.591, 0.071, 0]
    num_joints = 8
    [p.resetJointState(robot_id, idx, start_pose[idx]) for idx in range(num_joints)]
    for i in range (num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(joint_info)


    block_traj = []
    file_traj = "Robot_Sim/Block_traj_3_Blocks_.csv"
    with open(file_traj, 'r') as file:
        for line in file:
            row = literal_eval(line.strip())  # Convert the string to a list
            block_traj.append(row)
    
    for i,k in enumerate(list(objs.keys())):
        objs[k] = block_traj[i] + objs[k]
    
    create_3d_grid_world(objs, block_size, obs, block_mass, offset)
    
    ee_index = 8
    # file_path_pos = open("Robot_Sim/pick_place_demo_pos.txt","r")
    # file_path_ori = open("Robot_Sim/pick_place_demo_ori.txt","r")
    # demo_pos_reader = csv.reader(file_path_pos)
    # demo_ori_reader = csv.reader(file_path_ori)

    demo_waypoints, demo_orientations = load_demo_library_xlsx('Robot_Sim/demo_library.xlsx')

    traj = []
    
    
    file_traj = "Robot_Sim/End_effector_traj_2_Blocks_.csv"
    with open(file_traj, 'r') as file:
        for line in file:
            row = literal_eval(line.strip())  # Convert the string to a list
            traj.append(row)

    

    while True:
        x_ee = p.getLinkState(robot_id,ee_index)[0]
        print("The end effector is at: ",x_ee)
        pdb.set_trace()
        # Run the final trajectory
        for pt,traj_i in enumerate(traj):
            for pts in traj_i:
                pos_ee = pts[0:3]
                ori_ee = p.getQuaternionFromEuler(pts[3:])
                joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee,maxNumIterations=100)
                joint_angles = [0] + list(joint_angles)
                num_joints = len(joint_angles)
                [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
                p.resetBasePositionAndOrientation(2*(pt+1),[pos_ee[0],pos_ee[1]+0.05,pos_ee[2]],[0,0,0,1])
                x_ee = p.getLinkState(robot_id,ee_index)[0]
                # print("End effector is at: ", x_ee)
                time.sleep(0.1)
            
            print("Placed block",pt+1)
            pdb.set_trace()
            
        print("Finished!")

        # Stop over the block
        # for k in list(objs.keys()):
        #     obj_ = objs[k]
        #     pos = list(obj_[0])
        #     pos[2] = pos[2] + 0.15
        #     ori = [0,0.707,-0.707,0]
        #     joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos, ori, maxNumIterations=100)
        #     joint_angles = [0] + list(joint_angles)
        #     num_joints = len(joint_angles)
        #     [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
        #     x_ee = p.getLinkState(robot_id,ee_index)[0]
        #     print("End effector is at: ", x_ee)
        #     time.sleep(0.5)

        

        
        
        # for j,k in enumerate(list(objs.keys())):
        #     # demo_pos = objs[k][0]
        #     # print("Object is at: ", demo_pos)
        #     # demo_ori = p.getQuaternionFromEuler([0,0,0])

        #     # demo_pos = [0.5248635179676534, 0.05705548780077475, 0.09538625221514324]
        #     # demo_ori = [0.005769432876108959, -0.016932770229721575, 0.7063070706937895, 0.707679529748172]

        #     joint_angles = p.calculateInverseKinematics(robot_id,ee_index,demo_pos, demo_ori,maxNumIterations=100)
        #     joint_angles = [0] + list(joint_angles)
        #     num_joints = len(joint_angles)
        #     [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
        #     x_ee = p.getLinkState(robot_id,ee_index)[0]
        #     print("End effector is at: ", x_ee)
        #     time.sleep(0.5)


        # Run the demo trajectories
        # for i in range(len(demo_waypoints)):
        #     for j in range(len(demo_waypoints[i])):
        #         joint_angles = p.calculateInverseKinematics(robot_id,ee_index,demo_waypoints[i][j], demo_orientations[i][j],maxNumIterations=100)
        #         joint_angles = [0] + list(joint_angles)
        #         num_joints = len(joint_angles)
        #         [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
        #         x_ee = p.getLinkState(robot_id,ee_index)[0]
        #         print("End effector is at: ", x_ee)
        #         time.sleep(0.5)
        #     pdb.set_trace()

    # while True:
    #     update_block_state(robot_id, objs)
    # while (p.isConnected()):
    #     p.stepSimulation()
        