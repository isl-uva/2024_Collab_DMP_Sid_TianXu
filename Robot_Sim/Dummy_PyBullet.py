import pybullet as p
import pybullet_data
import time
import yaml 
import pdb
import numpy as np

from robots import ur5_simple, ur5_robotiq 

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

        color = [1,0,0]
        p.createMultiBody(baseMass=block_mass, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        p.changeVisualShape(box_id, -1, rgbaColor=color)
        objs[k] = [position] + objs[k]
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
    position[0] = position[0] + block_size/2
    position[1] = position[1] + block_size/2 - 0.07
    position[2] = position[2] - block_size/2
    position_ee = position
    rotation_ee = [0,0,0.707,0.707]
    joint_angle = robot._calculateIK(position_ee,rotation_ee)
    robot._sendPositionCommand(joint_angle)
    p.stepSimulation()
    time.sleep(1)

def push_block(robot,obj):
    position = list(obj[0])
    for j in range(200):       
        position[0] = position[0] + block_size/2
        position[1] = position[1] + block_size/2 - 0.07 + 0.001*j
        position[2] = position[2] - block_size/2
        position_ee = position
        rotation_ee = [0,0,0.707,0.707]
        joint_angle = robot._calculateIK(position_ee,rotation_ee)
        robot._sendPositionCommand(joint_angle)
        p.stepSimulation()
        time.sleep(0.05)
    print("Done")
        
    

            



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

    robot = ur5_simple.UR5_Simple()
    # robot = ur5_robotiq.UR5_Robotiq()
    robot.initialize()

    create_3d_grid_world(objs, block_size, obs, block_mass)

    for i,k in enumerate(objs.keys()):
        obj = objs[k]
        hit_blocks(robot,obj)
        time.sleep(2)
        robot.closeGripper()
        print("Gripper Close")
        time.sleep(2)
        push_block(robot, obj)
        time.sleep(2)
        robot.reset()

    # Keep the simulation running
    # while True:
    #     p.stepSimulation()
    #     hit_blocks(robot,objs)
    #     time.sleep(2)
    #     robot.closeGripper()
    #     print("Gripper Close")
    #     time.sleep(2)
    #     push_block(robot, objs)
    #     time.sleep(5)
    #     robot.reset()
