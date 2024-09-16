import pybullet as p
import pybullet_data
import time
import yaml 
import pdb
import numpy as np

from robots import ur5_simple, ur5_robotiq, panda, kuka

def create_3d_grid_world(objs, block_size, obs, block_mass):
    # Connect to the PyBullet physics server
    # p.connect(p.GUI)

    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    
    # # Set gravity
    # p.setGravity(0, 0, -9.81)
    
    # # Load the plane as the ground
    plane_id = p.loadURDF("plane.urdf")

    # # Setup Robot
    # cubeStartPos = [0,0,0]
    # cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    # robot_urdf = "/home/Siddharth/Siddharth/PyBullet/urdf/ur5/ur5_simple_gripper.urdf"
    # robot_id = p.loadURDF(robot_urdf, cubeStartPos, cubeStartOrientation,useFixedBase=True)
    # start_pose = [0., 0., -2.137, 1.432, -0.915, -1.591, 0.071]
    # num_joints = 7
    # [p.resetJointState(robot_id, idx, start_pose[idx]) for idx in range(num_joints)]
    


    

    ###############################################
    # Setup Blocks
    ###############################################

    for i,k in enumerate(list(objs.keys())):      
        # Create a box at the specified position with the specified color
        box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[block_size/2, block_size/2, block_size/2])
        # box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")
        position_ = objs[k][0]
        position = []
        for i,x in enumerate(position_):

            if i == 2:
                x = x*block_size + block_size/2
            elif i == 0:
                x = x*block_size/2 + block_size/2 +  0.5
            else:
                x = x*block_size/2 + block_size/2

            position.append(x)

        position = tuple(position)

        goal_position_ = objs[k][-1]
        goal_position = []
        for i,x in enumerate(goal_position_):

            if i == 2:
                x = x*block_size + block_size/2
            elif i == 0:
                x = x*block_size/2 + block_size/2 +  0.5
            else:
                x = x*block_size/2 + block_size/2

            goal_position.append(x)

        goal_position = tuple(goal_position)

        color = [1,0,0]
        p.createMultiBody(baseMass=block_mass, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        p.changeVisualShape(box_id, -1, rgbaColor=color)
        p.changeDynamics(box_id,
                         linkIndex=-1,
                         lateralFriction = 100,
                         rollingFriction = 0.00001)
                        #  restitution = 0.8,
                        #  contactStiffness = 10000,
                        #  contactDamping = 100)

        # startOrientation = [0,0,0,1]
        # p.resetBasePositionAndOrientation(box_id, position, startOrientation)
        objs[k] = [position,box_id] + objs[k] + [goal_position]
        # pdb.set_trace()
    





    ###############################################
    # Setup Obstacles
    ###############################################

    for i,ob in enumerate(obs):      
        # Create a box at the specified position with the specified color
        box_id = p.createCollisionShape(p.GEOM_BOX, halfExtents=[block_size/2, block_size/2, block_size/2])
        position_ = ob
        position = []
        for i,x in enumerate(position_):

            if i == 2:
                x = x*block_size + block_size/2
            elif i == 0:
                x = x*block_size/2 + block_size/2 + 0.5
            else:
                x = x*block_size/2 + block_size/2

            position.append(x)

        position = tuple(position)

        color = [0,0,0]
        p.createMultiBody(baseMass=0, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        p.changeVisualShape(box_id, -1, rgbaColor=color)


def hit_blocks(robot,obj):
    position = list(obj[0])
    print(position)
    position[0] = position[0] + block_size/2
    position[1] = position[1] + block_size/2 - 0.07
    position[2] = position[2] - block_size/2
    position_ee = position
    print("x_ee",position)
    rotation_ee = [0,0,0.707,0.707]
    joint_angle = robot._calculateIK(position_ee,rotation_ee)
    robot._sendPositionCommand(joint_angle)
    p.stepSimulation()
    time.sleep(1)

def push_block(robot,obj):
    blk_position = list(obj[0])
    print(blk_position)
    position = [0,0,0]
    for j in range(200):       
        position[0] = blk_position[0] + block_size/2
        position[1] = blk_position[1] + block_size/2 - 0.07 + 0.001*j
        position[2] = blk_position[2] - block_size/2
        position_ee = position
        rotation_ee = [0,0,0.707,0.707]
        joint_angle = robot._calculateIK(position_ee,rotation_ee)
        robot._sendPositionCommand(joint_angle)
        p.stepSimulation()
        time.sleep(0.05)
    print("Done")

def pick_block(robot,obj):
    import csv

    file_path = "pick_place_demo.csv"

    blk_position = list(obj[0])
    blk_position[0] = blk_position[0] + 0.01
    blk_position[1] = blk_position[1]  
    blk_position[2] = blk_position[2]  #+ 0.025
    print("Block is at:",blk_position)
    position_ee = blk_position
    rotation_ee = [0,0,0.707,0.707]
    joint_angle = robot._calculateIK(position_ee,rotation_ee)
    robot._sendPositionCommand(joint_angle)
    
    pdb.set_trace()
    ee_pose = p.getLinkState(robot.id,12)
    print("End Effector is at: ", ee_pose[0])
    time.sleep(2)
    demo_pos = ee_pose[0]
    demo_ori = ee_pose[1]
    
   


    # robot._sendGripperCommand(0.4,0.4)
    robot.controlGripper(0.2)    
    # robot.closeGripper()
    time.sleep(0.1)

    # Place Block
    position = [0,0,0]
    for j in range(300):       
        position[0] = blk_position[0] - 0.01
        position[1] = blk_position[1] 
        position[2] = blk_position[2]  + 0.001*j
        position_ee = position
        rotation_ee = [0,0,0.707,0.707]
        joint_angle = robot._calculateIK(position_ee,rotation_ee)
        robot._sendPositionCommand(joint_angle)
        # p.stepSimulation()
        # time.sleep(0.05)
    print("Done")
    time.sleep(2)
    ee_pose = p.getLinkState(robot.id,12)
    print("End Effector is at: ", ee_pose[0])
    demo_pos = demo_pos + ee_pose[0]
    demo_ori = demo_ori + ee_pose[1]
    

#     return demo_pos, demo_ori
    
        
        
# def place_obj(robot,obj):

    goal_position = list(obj[-1])
    print("Goal is: ",goal_position)

    goal_position[0] = goal_position[0] #+ block_size/2
    goal_position[1] = goal_position[1] #+ block_size/2 
    goal_position[2] = goal_position[2] - block_size/2 
    position_ee = goal_position
    print("x_ee",goal_position)
    rotation_ee = [0,0,0.707,0.707]
    joint_angle = robot._calculateIK(position_ee,rotation_ee)
    robot._sendPositionCommand(joint_angle)

    x_ee_pos = p.getLinkState(robot.id,12)[0]
    x_ee_ori = p.getLinkState(robot.id,12)[1]
    time.sleep(2)
    ee_pose = p.getLinkState(robot.id,12)
    print("End Effector is at: ", ee_pose[0])
    demo_pos = demo_pos + ee_pose[0]
    demo_ori = demo_ori + ee_pose[1]

    

    print("open")
    robot.openGripper()
    time.sleep(2)

    position = [0,0,0]
    curr_pos = position_ee
    for j in range(200):       
        position[0] = curr_pos[0] -0.01#+ block_size/2
        position[1] = curr_pos[1] #+ block_size/2 
        position[2] = curr_pos[2] - block_size/2  + 0.001*j
        position_ee = position
        rotation_ee = [0,0,0.707,0.707]
        joint_angle = robot._calculateIK(position_ee,rotation_ee)
        robot._sendPositionCommand(joint_angle)
        # p.stepSimulation()
        time.sleep(1/240.0)
    print("Done")
    time.sleep(2)
    ee_pose = p.getLinkState(robot.id,12)
    print("End Effector is at: ", ee_pose[0])
    demo_pos = demo_pos + ee_pose[0]
    demo_ori = demo_ori + ee_pose[1]

    

    return demo_pos, demo_ori



if __name__ == "__main__":
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

    # robot = ur5_simple.UR5_Simple()
    robot = ur5_robotiq.UR5_Robotiq()
    # robot = panda.Panda()
    # robot = kuka.Kuka()
    robot.initialize()

    create_3d_grid_world(objs, block_size, obs, block_mass)

    p.setTimeStep(1/120.0)
    p.setRealTimeSimulation(5)

   
    # for i,k in enumerate(objs.keys()):
    #     obj = objs[k]
    #     # hit_blocks(robot,obj)
    #     # time.sleep(1)
    #     # robot.closeGripper()
    #     # print("Gripper Close")
    #     # time.sleep(1)
    #     # push_block(robot, obj)
    #     # time.sleep(1)
    #     robot.reset()
    #     print("Robot Reset")
    #     time.sleep(1)
    #     pick_block(robot, obj)
    #     time.sleep(1)
    #     place_obj(robot,obj)
    #     time.sleep(1)

    timeStep = 1. / 240.
    while (p.isConnected()):
        p.stepSimulation()
        time.sleep(timeStep)
        for i,k in enumerate(objs.keys()):
            obj = objs[k]
            # hit_blocks(robot,obj)
            # time.sleep(1)
            # robot.closeGripper()
            # print("Gripper Close")
            # time.sleep(1)
            # push_block(robot, obj)
            # time.sleep(1)
            robot.reset()
            print("Robot Reset")
            time.sleep(1)
            demo_pos, demo_ori = pick_block(robot, obj)
            time.sleep(1)
            print("Positions are:\n",demo_pos)
            print("Orientations are:\n",demo_ori)
            pdb.set_trace()
            # demo_pos, demo_ori = place_obj(robot,obj)
            # time.sleep(1)
            # pdb.set_trace()