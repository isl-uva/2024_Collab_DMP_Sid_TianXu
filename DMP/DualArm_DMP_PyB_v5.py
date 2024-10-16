import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from io import BytesIO
import imageio

from dmp.dmp import DMPs_cartesian
from dmp.obstacle_superquadric import Obstacle_Dynamic as sq_dyn

from DMP_map_Rotation import rotation_matrix_from_vectors, tra_map_param, tra_map_forward, tra_map_inverse
from Minimal_Ellipsoid import rotate_ellipsoid, draw_ellipsoid, find_minimum_volume_ellipsoid,plot_ellipsoid_and_points

from GlobalPose import getlink_global_pose, get_semiaxes_Constants

import pybullet as pb
import pybullet_data
import time
import yaml
import pdb
from types import SimpleNamespace as SN

# from Robot_Sim.robots import ur5_robotiq
import sys
import os
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))

from Robot_Sim.robots.ur5_robotiq import UR5_Robotiq


# number of links
num_links = 6 

# Potential Field Dictionary
Ellipsoid_dict_agent1={}
Ellipsoid_dict_agent2={}


def plot3d_2(points):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Create a 3D plot
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # Scatter plot for the points
    ax.scatter(x, y, z, color='y', marker='+')
    ax.plot(x, y, z, color='k')

    # Optionally label the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # ax.set_xlim(-1.5, 1.5)
    # ax.set_ylim(-1.5, 1.5)
    # ax.set_zlim(-2, 2)

    # Show the plot
    # plt.show()

def plot3d_(points):
    x = points[:, 0]
    y = points[:, 1]
    z = points[:, 2]

    # Create a 3D plot
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # Scatter plot for the points
    ax.scatter(x, y, z, color='r', marker='o')
    ax.plot(x, y, z, color='b')

    # Optionally label the axes
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')
    # plt.axes()
    # Set equal scaling for all axes
    max_range = np.array([x.max()-x.min(), y.max()-y.min(), z.max()-z.min()]).max() / 2.0

    mid_x = (x.max()+x.min()) * 0.5
    mid_y = (y.max()+y.min()) * 0.5
    mid_z = (z.max()+z.min()) * 0.5

    ax.set_xlim(mid_x - max_range, mid_x + max_range)
    ax.set_ylim(mid_y - max_range, mid_y + max_range)
    ax.set_zlim(mid_z - max_range, mid_z + max_range)

    ax.set_xlim(-5, 5)
    ax.set_ylim(-5, 5)
    ax.set_zlim(-2, 2)

    # ax.set_xlim(-1.5, 1.5)
    # ax.set_ylim(-1.5, 1.5)
    # ax.set_zlim(-2, 2)

    # Show the plot
    # plt.show()

# Get global pose of each link
def getLinkFeatures(robot):
    Link_gloablpose = {}
    Link_globalvel = {}
    for l in range(robot.num_joints):
        LinkState = robot._getLinkStates(l)[0:2]
        Pos = LinkState[0]
        Ori = R.from_quat(LinkState[1])
        Rotatation_matrix = Ori.as_matrix()
        semi_axes_cons = get_semiaxes_Constants(f"Link{l}")
        Link_gloablpose[f"Link{l}"] = [Pos, Rotatation_matrix, semi_axes_cons]
        Link_globalvel[f"Link{l}"] = [robot._getLinkStates(l)[6]]

    return Link_gloablpose, Link_globalvel


# define potential field
def perturb_dynamic_agent1(x, v):    
    out =  0
    for i in range(1, num_links+1):
        key=f'link{i}'
        out+=Ellipsoid_dict_agent2[key].gen_external_force(x, v-Vel_dict_agent2[key])
    # print(out)
    return out

def perturb_dynamic_agent2(x, v):
    out = 0
    for i in range(1, num_links+1):
        key=f'link{i}'
        out+=Ellipsoid_dict_agent1[key].gen_external_force(x, v-Vel_dict_agent1[key])
    return out

# Define link data map mechanism
def Map_Link_Data(data_dict_agent1,data_dict_agent2,velocity_dict_agent1,velocity_dict_agent2):
# Create empty dictionaries to store results
    Center_dict_agent1 = {}
    Axes_dict_agent1 = {}
    Vel_dict_agent1 ={}
    
    Center_dict_agent2 = {}
    Axes_dict_agent2 = {}
    Vel_dict_agent2 ={}
    
    # Use enumerate to iterate over the items in x
    for i in range(1,num_links+1):
        key=f'link{i}'
        print(key, data_dict_agent1[key])
        
        #calculate mapped ellipsoid geometry of agent1
        center, axes, _ = find_minimum_volume_ellipsoid(data_dict_agent1[key])
        #map to the related coordination 
        center_map,axes_map,_ = rotate_ellipsoid(center, axes, alpha2,T2, R2)
        #calculate mapped ellipsoid velocity
        velocity=velocity_dict_agent1[key]
        velocity_map=alpha2 *R2 @ velocity
        velocity_map = velocity_map.T
    
        Center_dict_agent1[key]=center_map
        Axes_dict_agent1[key]=axes_map
        Vel_dict_agent1[key]=velocity_map
    
    
        #calculate mapped ellipsoid geometry of agent2
        center, axes, _ = find_minimum_volume_ellipsoid(data_dict_agent2[key])
        #map to the related coordination 
        center_map,axes_map,_ = rotate_ellipsoid(center, axes, alpha1,T1, R1)
        #calculate mapped ellipsoid velocity
        velocity=velocity_dict_agent2[key]
        velocity_map=alpha1 *R1 @ velocity
        velocity_map = velocity_map.T
    
        Center_dict_agent2[key]=center_map
        Axes_dict_agent2[key]=axes_map
        Vel_dict_agent2[key]=velocity_map

    return Center_dict_agent1, Axes_dict_agent1, Vel_dict_agent1, Center_dict_agent2, Axes_dict_agent2, Vel_dict_agent2

# Define link data update mechanism
def Update_link_Data(Center_dict_agent1, Axes_dict_agent1, Vel_dict_agent1, Center_dict_agent2, Axes_dict_agent2, Vel_dict_agent2):
    print('Arm Info Update')
    for i in range(1,num_links+1):
        key=f'link{i}'
        Ellipsoid_dict_agent1[key].center=Center_dict_agent1[key]
        # print(Ellipsoid_dict_agent1[key].center)
        # Ellipsoid_dict_agent1[key].center=np.array([0,0,0])
        Ellipsoid_dict_agent1[key].axis=Axes_dict_agent1[key]
        Ellipsoid_dict_agent2[key].center=Center_dict_agent2[key]
        Ellipsoid_dict_agent2[key].axis=Axes_dict_agent2[key]



#DMP 3D plot
def plotresult():
    fig=plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(gamma[:, 0], gamma[:, 1], gamma[:, 2],'--k', linewidth=1.5)
    ax.plot(gamma2[:, 0], gamma2[:, 1], gamma2[:, 2],'--k', linewidth=1.5)
    # ax.plot(gamma[:, 0], gamma[:, 1], gamma[:, 2],'-g', linewidth=1.5)
    ax.plot(gamma_tra[:, 0], gamma_tra[:, 1], gamma_tra[:, 2],'-r', linewidth=1.5)
    ax.plot(gamma2_tra[:, 0], gamma2_tra[:, 1], gamma2_tra[:, 2],'-r', linewidth=1.5)
    ax.plot_surface(x, y, z, color='b', alpha=0.3, edgecolor=(0, 0, 0, 0.1))  # Use alpha for transparency
    # ax.plot_surface(x_, y_, z_, color='r', alpha=0.3, edgecolor=(0, 0, 0, 0.1))  # Use alpha for transparency
    
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    
    ax.view_init(elev=10, azim=20)
    
    # Manually adjust the subplot margins
    plt.subplots_adjust(left=0.1, right=0.9, top=0.9, bottom=0.1)
    # plt.draw()         # Redraw the current figure
    # plt.pause(0.1) 

    buf = BytesIO()
    plt.savefig(buf, format="png")
    plt.close(fig)

    buf.seek(0)
    image = imageio.imread(buf)
    # writer.append_data(image)

    buf.close()


if __name__=="__main__":
    # pb.connect(pb.GUI)
    pb.connect(pb.DIRECT)
    pb.setGravity(0, 0, -9.81)
    pb.setAdditionalSearchPath(pybullet_data.getDataPath()) #used by loadURDF
    plane_id = pb.loadURDF("plane.urdf")

    '''
    Load Robots
    '''
    # robot1 = ur5_robotiq.UR5_Robotiq()
    # robot2 = ur5_robotiq.UR5_Robotiq()

    robot1 = UR5_Robotiq()
    robot2 = UR5_Robotiq()

    robot1.initialize(base_pos=[0.0, -0.0, 0.0])
    robot2.initialize(base_pos=[0.5, -0.0, 0.0])


    pb.setTimeStep(1/100)


    # Trajectory 2
    # Trajectory

    t_des = np.linspace(0.0, 1.0, 1000)

    '''
    Line Trajectory
    '''
    start1 = robot1._getLinkState(robot1.end_effector_index)[0]
    end1 = [0.0, 0.2, 0.2]
    start2 = robot2._getLinkState(robot2.end_effector_index)[0]
    end2 = [0.5, 0.2, 0.2]

    gamma = np.array([np.linspace(start1[i], end1[i], len(t_des)) for i in range(3)]).T
    gamma2 = np.array([np.linspace(start2[i], end2[i], len(t_des)) for i in range(3)]).T

    '''
    TODO: Make more examples of trajectories.
    '''

    alpha1,T1,R1=tra_map_param(gamma)
    gamma_map=tra_map_forward(gamma, alpha1, T1, R1)

    alpha2,T2,R2=tra_map_param(gamma2)
    gamma2_map=tra_map_forward(gamma2, alpha2, T2, R2)


    # call the function to obtain data_dict_agent1
    
    data_dict_agent1, velocity_dict_agent1 = getLinkFeatures(robot1)
    data_dict_agent2, velocity_dict_agent2 = getLinkFeatures(robot2)
    # plot3d_(data_dict_agent1['link3'])
    
    Center_dict_agent1, Axes_dict_agent1, Vel_dict_agent1, Center_dict_agent2, Axes_dict_agent2, Vel_dict_agent2= \
    Map_Link_Data(data_dict_agent1, data_dict_agent2, velocity_dict_agent1, velocity_dict_agent2)
    print(Axes_dict_agent1)

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # for i in range(1,3):
    #     key=f'link{i}'
    #     plot3d_(data_dict_agent1[key])      
    # plt.show()
    # pdb.set_trace()
        

    # for i in range(1,num_links+1):
    #     key=f'link{i}'
    #     plot_ellipsoid_and_points(Center_dict_agent1[key], Axes_dict_agent1[key], data_dict_agent1[key], ax)

    # plt.show()
    # pdb.set_trace()
    
    

    #DMP Ini
    K=1050.0
    alpha_s = 4.0
    tol=0.03
    dt=0.01

    MP = DMPs_cartesian(K=K, alpha_s=alpha_s, tol=tol)
    MP2= DMPs_cartesian(K=K, alpha_s=alpha_s, tol=tol)

    #DMP try
    # MP.imitate_path(t_des=t_des, x_des=gamma)
    # MP2.imitate_path(t_des=t_des, x_des=gamma2)

    MP.imitate_path(t_des=t_des, x_des=gamma_map)
    MP2.imitate_path(t_des=t_des, x_des=gamma2_map)

    gamma_, _, _, _ = MP.rollout()
    gamma2_, _, _, _ = MP2.rollout()
    t_des_ = np.linspace(0.0, 1.0, np.shape(gamma)[0])

    # Plot Ellipsoid (Need to be modified for better plot)
    # Define the ellipsoid parameters
    a = 0.2  # Semi-axis length in the x-direction
    b = 0.1  # Semi-axis length in the y-direction
    c = 0.15  # Semi-axis length in the z-direction

    # Create a grid of values for theta and phi
    theta = np.linspace(0, 2 * np.pi, 20)
    phi = np.linspace(0, np.pi, 10)
    theta, phi = np.meshgrid(theta, phi)

    # Parametric equations for the ellipsoid
    x = a * np.sin(phi) * np.cos(theta) + 0.0
    y = b * np.sin(phi) * np.sin(theta) + 1.0
    z = c * np.cos(phi) + 0.25

    MP.x_0 = gamma_[0]
    MP.x_goal = gamma_[-1]

    MP2.x_0 = gamma2_[0]
    MP2.x_goal = gamma2_[-1]

    MP.reset_state()
    MP2.reset_state()
    gamma_tra = np.array([gamma[0]])
    gamma2_tra= np.array([gamma2[0]])

    dyn_lambda = 6.74
    dyn_eta = 1.14
    dyn_beta = 0.55

    
    for i in range(1, num_links+1):
        center_map=Center_dict_agent1[f"link{i}"]
        axes_map=Axes_dict_agent1[f"link{i}"]
        Ellipsoid_dict_agent1[f"link{i}"]=sq_dyn(center=center_map, axis=axes_map, coeffs=np.array([1,1,1]), lmbda=dyn_lambda, beta=dyn_beta, eta=dyn_eta)


        center_map=Center_dict_agent2[f"link{i}"]
        axes_map=Axes_dict_agent2[f"link{i}"]
        Ellipsoid_dict_agent2[f"link{i}"]=sq_dyn(center=center_map, axis=axes_map, coeffs=np.array([1,1,1]), lmbda=dyn_lambda, beta=dyn_beta, eta=dyn_eta)

    plt.ion()

    desired_x = [[],[]]
    measured_x = [[],[]]

    desired_y = [[],[]]
    measured_y = [[],[]]

    desired_z = [[],[]]
    measured_z = [[],[]]

    flag_conv = False
    flag_conv2 = False
    total_time=0
    while total_time < 1:

        '''
        TODO: Change while loop condition.
        '''
        while not (flag_conv and flag_conv2):
            # obst_dynamic.center=obst_dynamic.center+v_e*dt
            total_time += dt
            
            # close_flag= np.linalg.norm(MP.x - MP2.x)>0.2
            close_flag=True

                # Parametric equations for the ellipsoid
            # x_ = a * np.sin(phi) * np.cos(theta) + obst_dynamic.center[0]
            # y_ = b * np.sin(phi) * np.sin(theta) + obst_dynamic.center[1]
            # z_ = c * np.cos(phi) + obst_dynamic.center[2]
            
            

            if not close_flag:
                print("agent1 stop")

            if (not flag_conv) and close_flag:
                # apply potential field
                MP.step(external_force=perturb_dynamic_agent1)
                flag_conv = np.linalg.norm(MP.x - MP.x_goal) < MP.tol
                # Map coordinations
                center_agent1_origin=tra_map_inverse(np.array(MP.x), alpha1, T1, R1)

        
                #store trajecory in world map
                gamma_tra = np.append(gamma_tra, np.array([center_agent1_origin]), axis=0)
                position_ee = gamma_tra[-1,:]
                rotation_ee = [0,0,0.707,0.707]
                # rotation_ee = [0.707,0,0,0.707]
                joint_angle = robot1._calculateIK(position_ee,rotation_ee)
                # robot1._sendPositionCommand(joint_angle)
                robot1._resetJointState(joint_angle)
                pb.stepSimulation()

                print("Robot 1 -> ",gamma_tra[-1])
                l = robot1._getLinkState(robot1.end_effector_index)

                desired_x[0].append(gamma_tra[-1][0])
                measured_x[0].append(l[0][0])

                desired_y[0].append(gamma_tra[-1][1])
                measured_y[0].append(l[0][1])

                desired_z[0].append(gamma_tra[-1][2])
                measured_z[0].append(l[0][2])

                meas1 = tra_map_forward(np.array(l[0]),alpha1, T1, R1)
                flag_conv = np.linalg.norm(meas1 - MP.x_goal) < MP.tol
            
                print(np.linalg.norm(meas1- MP.x_goal))

                # call the function to obtain data_dict_agent1

                data_dict_agent1, velocity_dict_agent1 = getLinkFeatures(robot1)
                data_dict_agent2, velocity_dict_agent2 = getLinkFeatures(robot2)
                Center_dict_agent1, Axes_dict_agent1, Vel_dict_agent1, Center_dict_agent2, Axes_dict_agent2, Vel_dict_agent2= \
                Map_Link_Data(data_dict_agent1, data_dict_agent2, velocity_dict_agent1, velocity_dict_agent2)
                Update_link_Data(Center_dict_agent1, Axes_dict_agent1, Vel_dict_agent1, Center_dict_agent2, Axes_dict_agent2, Vel_dict_agent2)
                for i in range(1,num_links+1):
                    key = f'link{i}'
                    points_ = tra_map_forward(data_dict_agent1[key],alpha2,T2,R2)

                    plot_ellipsoid_and_points(Center_dict_agent1[key], Axes_dict_agent1[key], points_, ax)
                    plot3d_(points_)
                    # plot3d_2(data_dict_agent1[key])
                plot3d_2(gamma_)

                plt.show()
                pdb.set_trace()



            

            if not flag_conv2:
                # apply potential field
                MP2.step(external_force=perturb_dynamic_agent2)
                flag_conv2 = np.linalg.norm(MP2.x - MP2.x_goal) < MP2.tol
                # Map coordinations
                center_agent2_origin=tra_map_inverse(np.array(MP2.x), alpha2, T2, R2)

                #store trajecory in world map
                gamma2_tra = np.append(gamma2_tra, np.array([center_agent2_origin]), axis=0)
                position_ee = gamma2_tra[-1,:]
                rotation_ee = [0,0,0.707,0.707]
                # rotation_ee = [0.707,0,0,0.707]
                joint_angle = robot2._calculateIK(position_ee,rotation_ee)
                robot2._sendPositionCommand(joint_angle)
                pb.stepSimulation()
                print("Robot 2 -> ",gamma2_tra[-1])
                
                l = robot2._getLinkState(robot2.end_effector_index)

                desired_x[1].append(gamma2_tra[-1][0])
                measured_x[1].append(l[0][0])

                desired_y[1].append(gamma2_tra[-1][1])
                measured_y[1].append(l[0][1])

                desired_z[1].append(gamma2_tra[-1][2])
                measured_z[1].append(l[0][2])

                meas2 =  tra_map_forward(np.array(l[0]),alpha2, T2, R2)
                flag_conv2 = np.linalg.norm(meas2- MP2.x_goal) < MP2.tol
                print(np.linalg.norm(meas2- MP2.x_goal))

                # call the function to obtain data_dict_agent1

                data_dict_agent1, velocity_dict_agent1 = getLinkFeatures(robot1)
                data_dict_agent2, velocity_dict_agent2 = getLinkFeatures(robot2)
                Center_dict_agent1, Axes_dict_agent1, Vel_dict_agent1, Center_dict_agent2, Axes_dict_agent2, Vel_dict_agent2= \
                Map_Link_Data(data_dict_agent1, data_dict_agent2, velocity_dict_agent1, velocity_dict_agent2)
                Update_link_Data(Center_dict_agent1, Axes_dict_agent1, Vel_dict_agent1, Center_dict_agent2, Axes_dict_agent2, Vel_dict_agent2)

            time.sleep(0.01)


            if flag_conv:
                print("agent1 achieve goal")

            if flag_conv2:
                print("agent2 achieve goal")

            
            # plotresult()

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot measured values
    ax.scatter(measured_x[0], measured_y[0], measured_z[0], color='blue', label='Measured', s=50)

    # Plot desired values
    ax.scatter(desired_x[0], desired_y[0], desired_z[0], color='red', label='Desired', s=50)


    # Plot measured values
    ax.scatter(measured_x[1], measured_y[1], measured_z[1], color='blue', label='Measured', s=50)

    # Plot desired values
    ax.scatter(desired_x[1], desired_y[1], desired_z[1], color='red', label='Desired', s=50)

    # Add labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')


    # x_min = start1[0]
    # x_max = end2[0]
    # y_min = start1[1]
    # y_max = end2[1]
    # z_min = start1[2]
    # z_max = end2[2]
    # max_range = 0.05
    
    # ax.set_xlim(x_min - max_range, x_max + max_range)
    # ax.set_ylim(y_min - max_range, y_max + max_range)
    # ax.set_zlim(z_min - max_range, z_max + max_range)

    # Add a legend
    ax.legend()

    # Show plot
    plt.show()
    print(total_time)
    pdb.set_trace()
