import numpy as np
import pybullet as pb
import pybullet_data
import time
import yaml
import pdb
from types import SimpleNamespace as SN

from robots import ur5_robotiq


def create_3d_grid_world(objs, block_size, obs, block_mass):
    for i,k in enumerate(list(objs.keys())):      
        # Create a box at the specified position with the specified color
        box_id = pb.createCollisionShape(pb.GEOM_BOX, halfExtents=[block_size/2, block_size/2, block_size/2])
        
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
        pb.createMultiBody(baseMass=block_mass, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        pb.changeVisualShape(box_id, -1, rgbaColor=color)
        objs[k] = [position] + objs[k]

def goto_block(robots,objs_pos):
    # pdb.set_trace()
    blk_position1 = list(objs_pos[1])
    print(blk_position1)
    blk_position2 = list(objs_pos[0])
    print(blk_position2)

    position1 = [0,0,0]
    position2 = [0,0,0]
    for j in range(200):       
        position1[0] = blk_position1[0] + block_size/2 
        position1[1] = blk_position1[1] + block_size/2 - 0.25 + 0.01*j
        position1[2] = blk_position1[2] - block_size/2
        position_ee = position1
        rotation_ee = [0,0,0.707,0.707]
        joint_angle = robots[0]._calculateIK(position_ee,rotation_ee)
        robots[0]._sendPositionCommand(joint_angle)

        position2[0] = blk_position2[0] + block_size/2 
        position2[1] = blk_position2[1] + block_size/2 - 0.15 + 0.001*j
        position2[2] = blk_position2[2] - block_size/2
        position_ee = position2
        rotation_ee = [0,0,0.707,0.707]
        joint_angle = robots[1]._calculateIK(position_ee,rotation_ee)
        robots[1]._sendPositionCommand(joint_angle)
        pb.stepSimulation()
        l = pb.getLinkState(robot1.id,6)
        print(l)
        time.sleep(0.05)
    print("Done")



if __name__=="__main__":
    pb.connect(pb.GUI)
    # pb.connect(pb.DIRECT)
    pb.setGravity(0, 0, -9.81)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    plane_id = pb.loadURDF("plane.urdf")


    '''
    Load Objects
    '''
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

    create_3d_grid_world(objs, block_size, obs, block_mass)

    '''
    Load Robots
    '''
    robot1 = ur5_robotiq.UR5_Robotiq()
    robot2 = ur5_robotiq.UR5_Robotiq()

    robot1.initialize(base_pos=[0.0,  0.5, 0.0])
    robot2.initialize(base_pos=[0.0, -0.5, 0.0])

    objs_pos = []
    for k in list(objs.keys()):
        objs_pos.append(objs[k][0])
    print(objs_pos)
    goto_block(robots=[robot1,robot2], objs_pos=objs_pos)

    # while True:
    #     pb.stepSimulation()
    #     time.sleep(0.1)
    