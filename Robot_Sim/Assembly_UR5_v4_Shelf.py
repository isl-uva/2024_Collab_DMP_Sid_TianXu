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


COLORS = {}
COLORS['red'] = [1,0,0,1]
COLORS['green'] = [0,1,0,1]
COLORS['blue'] = [0,0,1,1]
COLORS['orange'] = [1,0.5,0,1]
COLORS['purple'] = [1,0,0.5,1]
COLORS['pink'] = [0.49,0.2,0.2,1]

def create_3d_grid_world(objs, block_size, obs, block_mass, offset): 

    ###############################################
    # Setup Blocks
    ###############################################
    blks_plced = []
    for i,kt in enumerate(Traj_data):
        if kt[0] in list(objs.keys()) and kt[0] not in blks_plced:
            objs[kt[0]] = [kt[1][0]] + objs[kt[0]]
            blks_plced.append(kt[0])
    
    for i,k in enumerate(list(objs.keys())):      
        # Create a box at the specified position with the specified color

        box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")
        position = objs[k][0]
        # position_ = objs[k][1]
        # position = [x*block_size + block_size/2 + offset[i] for i,x in enumerate(position_)]

        position = tuple(position)
        print(k," is at",position)
        
        p.createMultiBody(baseMass=block_mass, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        p.changeVisualShape(box_id, -1,-1, rgbaColor = COLORS[objs[k][-1]])
        
        startOrientation = [0.707,0,0,0.707]
        p.resetBasePositionAndOrientation(box_id, position, startOrientation)

        # Plot goal position
        goal_position_ = objs[k][1]
        goal_position = []
        goal_position = [x*block_size + block_size/2 + offset[i] for i,x in enumerate(goal_position_)]
        goal_position = tuple(goal_position)
        # goal_position = Traj_data[k][1]

        sphereRadius = 0.025
        spherecolor = COLORS[objs[k][-1]]
        spherecolor[-1] = 0.3
        sphereVisualShapeId = p.createVisualShape(shapeType=p.GEOM_SPHERE,
                                                radius=sphereRadius,
                                                visualFramePosition=[0, 0, 0],
                                                rgbaColor=spherecolor)

        # Create the sphere body
        sphereStartPos = tuple(goal_position)
        sphereStartOrientation = p.getQuaternionFromEuler([0, 0, 0])
        sphereId = p.createMultiBody(baseVisualShapeIndex=sphereVisualShapeId,
                                    basePosition=sphereStartPos,
                                    baseOrientation=sphereStartOrientation)

        objs[k] = [box_id] + objs[k] + [sphereId]
    


    ###############################################
    # Setup Obstacles
    ###############################################

    for i,ob in enumerate(obs):      

        # Create a box at the specified position with the specified color
        box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")
        position_ = ob
        # pdb.set_trace()
        position = [x*block_size + block_size/2 + offset[i] for i,x in enumerate(position_)]

        position = tuple(position)

        color = [0,0,0]
        p.createMultiBody(baseMass=block_mass, baseCollisionShapeIndex=box_id, basePosition=position, baseOrientation=[0, 0, 0, 1], baseVisualShapeIndex=box_id)
        p.changeVisualShape(box_id, -1,-1, rgbaColor = color)

        startOrientation = [0,0,0,1]
        p.resetBasePositionAndOrientation(box_id, position, startOrientation)


def reset_():
    for i,k in enumerate(list(objs.keys())):      
        # Create a box at the specified position with the specified color

        box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")
        position = objs[k][1]

        position = tuple(position)
                
        startOrientation = [0,0,0,1]
        p.resetBasePositionAndOrientation(objs[k][0], position, startOrientation)

    start_pose = [0., 0., -2.137, 1.432, -0.915, -1.591, 0.071, 0]
    num_joints = 8
    [p.resetJointState(robot_id, idx, start_pose[idx]) for idx in range(num_joints)]

        


def init_setup_robot():
    # Connect to the PyBullet physics server
    p.connect(p.GUI)
    # p.connect(p.DIRECt)

    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    
    # # Set gravity
    p.setGravity(0, 0, -9.81)
    
    # # Load the plane as the ground
    plane_id = p.loadURDF("plane.urdf")

    table_vis_id = p.createVisualShape(shapeType=p.GEOM_BOX,halfExtents=[0.5,0.8,0.06],visualFramePosition=[0, 0, 0], rgbaColor=[0.6, 0.4, 0, 1])
    tableId = p.createMultiBody(baseVisualShapeIndex=table_vis_id,
                                    basePosition=[0.5,0,0],
                                    baseOrientation=[0,0,0,1])

    
    
    '''
    Show Demo for Shelving
    '''
    shelf_id = p.loadURDF("Robot_Sim/urdf/shelf.urdf")
    p.resetBasePositionAndOrientation(shelf_id, posObj=[0.5,0.5,0.06], ornObj=[0, 0, 0.707, 0.707]) #ornObj =[0, 0 , 0 , 1])#
    
    book_pos = []
    position_ = [0.25,0.35,0.35]
    box_id = p.loadURDF("Robot_Sim/urdf/object/box/book.urdf")

    position = [x + block_size/2  for i,x in enumerate(position_)]
    blk_position = tuple(position)
    print(box_id,"is at",blk_position) 
    
    book_pos.append(blk_position)       
    
    p.resetBasePositionAndOrientation(box_id,posObj=blk_position,ornObj=[0,0,0.707,0.707])
    p.changeVisualShape(box_id, -1,-1, rgbaColor = (1,0,0,1))

    position_ = [0.25+0.055,0.35,0.35]
    box_id = p.loadURDF("Robot_Sim/urdf/object/box/book.urdf")

    position = [x + block_size/2  for i,x in enumerate(position_)]
    blk_position = tuple(position)
    print(box_id,"is at",blk_position)  
    book_pos.append(blk_position)        
    p.resetBasePositionAndOrientation(box_id,posObj=blk_position,ornObj=[0,0,0.707,0.707])
    p.changeVisualShape(box_id, -1,-1, rgbaColor = (1,0.5,0,1))

    position_ = [0.25+0.055,0.35,0.09]
    box_id = p.loadURDF("Robot_Sim/urdf/object/box/book.urdf")

    position = [x + block_size/2  for i,x in enumerate(position_)]
    blk_position = tuple(position)
    print(box_id,"is at",blk_position)
    book_pos.append(blk_position)          
    p.resetBasePositionAndOrientation(box_id,posObj=blk_position,ornObj=[0,0,0.707,0.707])
    p.changeVisualShape(box_id, -1,-1, rgbaColor = (0,1,0,1))

    return book_pos
    


    '''
    Show Demo for Stacking
    '''
    # position_ = [0.25,0.0,0.06]
    # box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")

    # position = [x + block_size/2  for i,x in enumerate(position_)]
    # blk_position = tuple(position)
    # print(box_id,"is at",blk_position)        
    # p.resetBasePositionAndOrientation(box_id,posObj=blk_position,ornObj=[0,0,0.707,0.707])
    # p.changeVisualShape(box_id, -1,-1, rgbaColor = (1,0.5,0,1))

    # position_ = [0.25,0.0,0.06+0.055]
    # box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")

    # position = [x + block_size/2  for i,x in enumerate(position_)]
    # blk_position = tuple(position)
    # print(box_id,"is at",blk_position)        
    # p.resetBasePositionAndOrientation(box_id,posObj=blk_position,ornObj=[0,0,0.707,0.707])
    # p.changeVisualShape(box_id, -1,-1, rgbaColor = (0,1,0,1))

    # position_ = [0.25,0.0,0.06+0.11]
    # box_id = p.loadURDF("Robot_Sim/urdf/object/box/box.urdf")

    # position = [x + block_size/2  for i,x in enumerate(position_)]
    # blk_position = tuple(position)
    # print(box_id,"is at",blk_position)        
    # p.resetBasePositionAndOrientation(box_id,posObj=blk_position,ornObj=[0,0,0.707,0.707])
    # p.changeVisualShape(box_id, -1,-1, rgbaColor = (1,0,0,1))




    
if __name__=="__main__":
    from types import SimpleNamespace as SN
    import yaml
    with open('HL_LfD_Combine/Block_stacking_HL.yaml', 'r') as f:
    # with open('HL_LfD_Combine/Block_pyramid.yaml', 'r') as f:
    # with open('HL_LfD_Combine/Around_wall.yaml', 'r') as f:
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
    task = config.env['Task']


    # robot_id = init_setup_robot()
    book_pos = init_setup_robot()
    

    robot_urdf = "Robot_Sim/urdf/ur5/ur5.urdf"
    # robot_urdf = "Robot_Sim/urdf/ur5/ur5_robotiq_85_gripper_fake.urdf"
    # pdb.set_trace()
    # robot_urdf = "Robot_Sim/kortex_description/arms/gen3/7dof/urdf/GEN3-7DOF-VISION_ARM_URDF_V12.urdf"

    # robot_urdf = "Robot_Sim/urdf/franka_panda/panda.urdf"
    cubeStartPos = [0,0,offset[-1]]
    cubeStartOrientation = p.getQuaternionFromEuler([0,0,0])
    robot_id = p.loadURDF(robot_urdf, cubeStartPos, cubeStartOrientation,useFixedBase=True)
    # start_pose = [0., 0., -2.137, 1.432, -0.915, -1.591, 0.071, 0]
    start_pose = [0., 0., -2.537, 1.432, -0.915, -1.591, 0.071, 0]
    num_joints = 8
    [p.resetJointState(robot_id, idx, start_pose[idx]) for idx in range(num_joints)]
    for i in range (num_joints):
        joint_info = p.getJointInfo(robot_id, i)
        print(joint_info)
    
    # file_name = "Robot_Sim/End_effector_traj_"+str(len(objs))+"_Blocks_.json"
    # file_traj = open("Robot_Sim/Sep_End_effector_traj_"+task+str(len(objs))+"_Blocks_.json")
        
    file_traj = open('Robot_Sim/End_effector_traj_'+task+str(len(objs))+'_Blocks_.json')
    Traj_data = json.load(file_traj)   
    
    # Setup the world and objects
    # create_3d_grid_world(objs, block_size, obs, block_mass, offset)
    # pdb.set_trace()


    # Start Simulation
    ee_index = 8

    p.resetDebugVisualizerCamera(cameraDistance=2, cameraYaw=50, cameraPitch=-25, cameraTargetPosition=[0.15,0.1,0])
    pdb.set_trace()
    
    while True:
        
        # print("The end effector is at: ",x_ee) 
        # Run the final trajectory
        for i, box in enumerate(book_pos):
            print("Book ",i, "is at: ", box)

        x_ee = p.getLinkState(robot_id,ee_index)[0]
        start = x_ee
        end = list(box)
        end[1] = end[1] - 0.05
        end[2] = end[2] + 0.025
        for i in range(11):
            pos_ee_ = i/10*np.array(end) + (1-i/10)*np.array(start)
            pos_ee = pos_ee_.tolist()
            ori_ee_euler = [1.57, 0.0, -3.14]
            # ori_ee_euler.reverse()
            ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
            joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee,maxNumIterations=100)
            joint_angles = [0] + list(joint_angles)
            num_joints = len(joint_angles)
            [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
            time.sleep(0.1)

        x_ee = p.getLinkState(robot_id,ee_index)[0]
        start = x_ee
        end = list(box)
        end[1] = end[1] - 0.25
        end[2] = end[2] + 0.025 + 0.25
        for i in range(11):
            pos_ee_ = i/10*np.array(end) + (1-i/10)*np.array(start)
            pos_ee = pos_ee_.tolist()
            ori_ee_euler = [1.57, 0.0, -3.14]
            # ori_ee_euler.reverse()
            ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
            joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee,maxNumIterations=100)
            joint_angles = [0] + list(joint_angles)
            num_joints = len(joint_angles)
            [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
            p.resetBasePositionAndOrientation(5,[pos_ee[0],pos_ee[1]+0.05,pos_ee[2]],[0,0,0.707,0.707])
            time.sleep(0.1)

        x_ee = p.getLinkState(robot_id,ee_index)[0]
        start = x_ee
        end = list(box)
        end[0] = end[0] + 0.055
        end[1] = end[1] - 0.055 
        end[2] = end[2] + 0.025 + 0.25
        for i in range(11):
            pos_ee_ = i/10*np.array(end) + (1-i/10)*np.array(start)
            pos_ee = pos_ee_.tolist()
            ori_ee_euler = [1.57, 0.0, -3.14]
            # ori_ee_euler.reverse()
            ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
            joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee,maxNumIterations=100)
            joint_angles = [0] + list(joint_angles)
            num_joints = len(joint_angles)
            [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
            p.resetBasePositionAndOrientation(5,[pos_ee[0],pos_ee[1]+0.05,pos_ee[2]],[0,0,0.707,0.707])
            time.sleep(0.1)
        
        print('+++++++++++++++>',end)
        pdb.set_trace()

        

    #     # p.startStateLogging(p.STATE_LOGGING_VIDEO_MP4,'Stack_2x2.mp4')

    #     for j,kt in enumerate(Traj_data):
    #         # Get to the block
    #         Last_position = p.getLinkState(robot_id,ee_index)[0]
    #         if j == 8:
    #             block_pos = kt[1][0]
    #             block_pos[2] = block_pos[2] + 0.1
    #             for i in range(6):               
    #                 pos_ee_ = i/5*np.array(block_pos) + (1-i/5)*np.array(Last_position)
    #                 pos_ee = pos_ee_.tolist()
    #                 ori_ee_euler = [1.57, 0.0, -3.14]
    #                 ori_ee_euler.reverse()
    #                 ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
    #                 joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee,maxNumIterations=100)
    #                 joint_angles = [0] + list(joint_angles)
    #                 num_joints = len(joint_angles)
    #                 [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
    #                 time.sleep(0.1)

            
    #             x_ee = p.getLinkState(robot_id,ee_index)[0]
    #             for i in range(5):               
    #                 pos_ee = list(x_ee)
    #                 pos_ee[2] = pos_ee[2] - 0.01*i
    #                 ori_ee_euler = [1.57, 0.0, -3.14]
    #                 ori_ee_euler.reverse()
    #                 ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
    #                 joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee,maxNumIterations=100)
    #                 joint_angles = [0] + list(joint_angles)
    #                 num_joints = len(joint_angles)
    #                 [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
    #                 time.sleep(0.1)


    #             obj_k = kt[0]     # Name of the block
    #             traj_i = kt[-1]   # Trajectory of the end effector
    #             for pts in traj_i:
    #                 pos_ee = pts[0:3]
    #                 pos_ee[2] = pos_ee[2] + 0.25
    #                 ori_ee_euler = pts[3:]
    #                 ori_ee_euler.reverse()
    #                 ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
                    
    #                 joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee, maxNumIterations=100)
    #                 joint_angles = [0] + list(joint_angles)
    #                 num_joints = len(joint_angles)
                    
    #                 [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
    #                 if k!= 'Trans':
    #                     p.resetBasePositionAndOrientation(objs[obj_k][0],[pos_ee[0],pos_ee[1],pos_ee[2]-block_size],[0,0,0,1])
    #                 x_ee = p.getLinkState(robot_id,ee_index)[0]
    #                 # print("End effector is at: ", x_ee)
    #                 time.sleep(0.1)

    #                 #Place the object
    #             for i in range(21):
    #                 pos_ee = pts[0:3]
    #                 pos_ee[2] = pos_ee[2] + 0.25 - 0.01*i
    #                 ori_ee_euler = pts[3:]
    #                 ori_ee_euler.reverse()
    #                 ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
                    
    #                 joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee, maxNumIterations=100)
    #                 joint_angles = [0] + list(joint_angles)
    #                 num_joints = len(joint_angles)
                    
    #                 [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
                    
    #                 if k!= 'Trans':
    #                     p.resetBasePositionAndOrientation(objs[obj_k][0],[pos_ee[0],pos_ee[1],pos_ee[2]-block_size],[0,0,0,1])
    #                 x_ee = p.getLinkState(robot_id,ee_index)[0]
    #                 time.sleep(0.1)

    #             Last_position = pos_ee
    #             print("Placed %f at %f",kt[0],kt[1][1])


    #         else:
    #             block_pos = kt[1][0]
    #             block_pos[2] = block_pos[2] + 0.1
    #             for i in range(6):               
    #                 pos_ee_ = i/5*np.array(block_pos) + (1-i/5)*np.array(Last_position)
    #                 pos_ee = pos_ee_.tolist()
    #                 ori_ee_euler = [1.57, 0.0, -3.14]
    #                 ori_ee_euler.reverse()
    #                 ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
    #                 joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee,maxNumIterations=100)
    #                 joint_angles = [0] + list(joint_angles)
    #                 num_joints = len(joint_angles)
    #                 [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
    #                 time.sleep(0.1)

            
    #             x_ee = p.getLinkState(robot_id,ee_index)[0]
    #             for i in range(5):               
    #                 pos_ee = list(x_ee)
    #                 pos_ee[2] = pos_ee[2] - 0.01*i
    #                 ori_ee_euler = [1.57, 0.0, -3.14]
    #                 ori_ee_euler.reverse()
    #                 ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
    #                 joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee,maxNumIterations=100)
    #                 joint_angles = [0] + list(joint_angles)
    #                 num_joints = len(joint_angles)
    #                 [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
    #                 time.sleep(0.1)


    #             obj_k = kt[0]     # Name of the block
    #             traj_i = kt[-1]   # Trajectory of the end effector

    #             for pts in traj_i:
    #                 pos_ee = pts[0:3]
    #                 pos_ee[2] = pos_ee[2] + 0.1
    #                 ori_ee_euler = pts[3:]
    #                 ori_ee_euler.reverse()
    #                 ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
                    
    #                 joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee, maxNumIterations=100)
    #                 joint_angles = [0] + list(joint_angles)
    #                 num_joints = len(joint_angles)
                    
    #                 [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]

    #                 if k!= 'Trans':
    #                     p.resetBasePositionAndOrientation(objs[obj_k][0],[pos_ee[0],pos_ee[1],pos_ee[2]-block_size],[0,0,0,1])
    #                 x_ee = p.getLinkState(robot_id,ee_index)[0]
    #                 # print("End effector is at: ", x_ee)
    #                 time.sleep(0.1)


    #             #Place the object
    #             for i in range(6):
    #                 pos_ee = pts[0:3]
    #                 pos_ee[2] = pos_ee[2] + 0.1 - 0.01*i
    #                 ori_ee_euler = pts[3:]
    #                 ori_ee_euler.reverse()
    #                 ori_ee = p.getQuaternionFromEuler(ori_ee_euler)
                    
    #                 joint_angles = p.calculateInverseKinematics(robot_id,ee_index,pos_ee, ori_ee, maxNumIterations=100)
    #                 joint_angles = [0] + list(joint_angles)
    #                 num_joints = len(joint_angles)
                    
    #                 [p.resetJointState(robot_id, idx, joint_angles[idx]) for idx in range(0,num_joints)]
                    
    #                 if k!= 'Trans':
    #                     p.resetBasePositionAndOrientation(objs[obj_k][0],[pos_ee[0],pos_ee[1],pos_ee[2]-block_size],[0,0,0,1])
    #                 x_ee = p.getLinkState(robot_id,ee_index)[0]
    #                 time.sleep(0.1)

    #             Last_position = pos_ee
    #             print("Placed %f at %f",kt[0],kt[1][1])
    #         # pdb.set_trace()
        
        
    #     time.sleep(1) # End of Demo
    #     reset_()
            
    #     print("Finished!")
    #     # p.stopStateLogging(p.STATE_LOGGING_VIDEO_MP4)
    #     pdb.set_trace()
        
        