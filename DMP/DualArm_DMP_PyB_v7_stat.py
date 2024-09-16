import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.spatial.transform import Rotation as R

from io import BytesIO
import imageio

from dmp.dmp import DMPs_cartesian
# from dmp.obstacle_superquadric import Obstacle_Dynamic as sq_dyn
from dmp.obstacle_superquadric import Obstacle_Static as sq_stat

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
num_links = 7 

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
    Link_gloablpos = {}
    Link_globalvel = {}
    Link_globalrot = {}
    Link_axes = {}
    
    for l in range(1,8):
        if l < 7:
            LinkState = robot._getLinkState(l)[0:2]
            Pos = LinkState[0]
            Ori = R.from_quat(LinkState[1])
            Rotatation_matrix = Ori.as_matrix()
            semi_axes_cons = get_semiaxes_Constants(f"link{l}")
            Link_gloablpos[f"link{l}"] = np.array(Pos)
            Link_axes[f"link{l}"] = np.array(semi_axes_cons)
            Link_globalrot[f"link{l}"]= np.array(Rotatation_matrix).T
            Link_globalvel[f"link{l}"] = np.array(robot._getLinkState(l)[6])
        else:
            l = 10
            LinkState = robot._getLinkState(l)[0:2]
            Pos = LinkState[0]
            Ori = R.from_quat(LinkState[1])
            Rotatation_matrix = Ori.as_matrix()
            semi_axes_cons = get_semiaxes_Constants(f"link{7}")
            Link_gloablpos[f"link7"] = np.array(Pos)
            Link_axes[f"link7"] = np.array(semi_axes_cons)
            Link_globalrot[f"link7"]= np.array(Rotatation_matrix).T
            Link_globalvel[f"link7"] = np.array(robot._getLinkState(l)[6])


    # print(Link_gloablpos) 
    # print(Link_axes)
    # print(Link_globalvel)
    # print(Link_globalrot)
    # pdb.set_trace()
    # cent_agent1, axes_agent1,velo_agent1,rotm_agent1
    return Link_gloablpos, Link_axes, Link_globalvel, Link_globalrot



# define potential field
def perturb_dynamic_agent1(x, v):    
    out =  0
    for i in range(1, num_links+1):
        key=f'link{i}'
        # out+=Ellipsoid_dict_agent2[key].gen_external_force_rotate(x, v-alpha1*R1@velo_agent2[key], R1@rotm_agent2[key])
        out+=Ellipsoid_dict_agent2[key].gen_external_force_rotate(x, R1@rotm_agent2[key])
    # print("Perturbance 1", out)
    return out

def perturb_dynamic_agent2(x, v):
    out = 0
    for i in range(1, num_links+1):
        key=f'link{i}'
        # out+=Ellipsoid_dict_agent1[key].gen_external_force_rotate(x, v-alpha2*R2@velo_agent1[key], R2@rotm_agent1[key])
        out+=Ellipsoid_dict_agent1[key].gen_external_force_rotate(x, R2@rotm_agent1[key])
        # print('local',key, Ellipsoid_dict_agent1[key].gen_external_force_rotate(x, v-alpha2*R2@velo_agent1[key], R2@rotm_agent1[key]))
    # print("Perturbance 2", out)
    return out



# Define link data update mechanism
def Update_link_Data(cent_agent1, cent_agent2):
    print('Arm Info Update')
    for i in range(1,num_links+1):
        key = f"link{i}"


        # center_map=cent_agent1[f"link{i}"]
        center_map=cent_agent1[key]
        center_map=tra_map_forward(center_map, alpha2, T2, R2)
        Ellipsoid_dict_agent1[key].center = center_map

        center_map=cent_agent2[key]
        center_map=tra_map_forward(center_map, alpha1, T1, R1)
        Ellipsoid_dict_agent2[key].center=center_map



#DMP 3D plot
def plotresult(writer):
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
    writer.append_data(image)

    buf.close()

def get_3d_plot2():
     # Create a 3D figure
    fig = plt.figure(figsize=(12, 8))
    # ax = fig.add_subplot(111, projection='3d')


    x1_positions_m, y1_positions_m, z1_positions_m = measured_x[0], measured_y[0], measured_z[0]
    split_1 = len(x1_positions_m) // 3
    split_2 = 2 * len(x1_positions_m) // 3
    x1_part1_m, y1_part1_m, z1_part1_m = x1_positions_m[:split_1], y1_positions_m[:split_1], z1_positions_m[:split_1]
    x1_part2_m, y1_part2_m, z1_part2_m = x1_positions_m[:split_2], y1_positions_m[:split_2], z1_positions_m[:split_2]
    x1_part3_m, y1_part3_m, z1_part3_m = x1_positions_m, y1_positions_m, z1_positions_m

    x1_positions_d, y1_positions_d, z1_positions_d = desired_x[0], desired_y[0], desired_z[0]
    split_1 = len(x1_positions_d) // 3
    split_2 = 2 * len(x1_positions_d) // 3
    x1_part1_d, y1_part1_d, z1_part1_d = x1_positions_d[:split_1], y1_positions_d[:split_1], z1_positions_d[:split_1]
    x1_part2_d, y1_part2_d, z1_part2_d = x1_positions_d[:split_2], y1_positions_d[:split_2], z1_positions_d[:split_2]
    x1_part3_d, y1_part3_d, z1_part3_d = x1_positions_d, y1_positions_d, z1_positions_d

    x2_positions_m, y2_positions_m, z2_positions_m = measured_x[1], measured_y[1], measured_z[1]
    split_1 = len(x1_positions_m) // 3
    split_2 = 2 * len(x1_positions_m) // 3
    x2_part1_m, y2_part1_m, z2_part1_m = x2_positions_m[:split_1], y2_positions_m[:split_1], z2_positions_m[:split_1]
    x2_part2_m, y2_part2_m, z2_part2_m = x2_positions_m[:split_2], y2_positions_m[:split_2], z2_positions_m[:split_2]
    x2_part3_m, y2_part3_m, z2_part3_m = x2_positions_m, y2_positions_m, z2_positions_m

    x2_positions_d, y2_positions_d, z2_positions_d = desired_x[1], desired_y[1], desired_z[1]
    split_1 = len(x1_positions_d) // 3
    split_2 = 2 * len(x1_positions_d) // 3
    x2_part1_d, y2_part1_d, z2_part1_d = x2_positions_d[:split_1], y2_positions_d[:split_1], z2_positions_d[:split_1]
    x2_part2_d, y2_part2_d, z2_part2_d = x2_positions_d[:split_2], y2_positions_d[:split_2], z2_positions_d[:split_2]
    x2_part3_d, y2_part3_d, z2_part3_d = x2_positions_d, y2_positions_d, z2_positions_d

    ref = np.array(gamma).T
    k = 25
    x1_ref, y1_ref, z1_ref = ref[0,::k].tolist(), ref[1,::k].tolist(), ref[2,::k].tolist()
    split_1 = len(x1_ref) // 3
    split_2 = 2 * len(x1_ref) // 3
    x1_ref1, y1_ref1, z1_ref1 = x1_ref[:split_1], y1_ref[:split_1], z1_ref[:split_1]
    x1_ref2, y1_ref2, z1_ref2 = x1_ref[:split_2], y1_ref[:split_2], z1_ref[:split_2]
    x1_ref3, y1_ref3, z1_ref3 = x1_ref, y1_ref, z1_ref

    ref2 = np.array(gamma2).T
    x2_ref, y2_ref, z2_ref = ref2[0,::k].tolist(), ref2[1,::k].tolist(), ref2[2,::k].tolist()
    split_1 = len(x2_ref) // 3
    split_2 = 2 * len(x2_ref) // 3
    x2_ref1, y2_ref1, z2_ref1 = x2_ref[:split_1], y2_ref[:split_1], z2_ref[:split_1]
    x2_ref2, y2_ref2, z2_ref2 = x2_ref[:split_2], y2_ref[:split_2], z2_ref[:split_2]
    x2_ref3, y2_ref3, z2_ref3 = x2_ref, y2_ref, z2_ref

    
    x_max = np.array(desired_x[0]).max()+0.1
    y_max = np.array(desired_y[0]).max()+0.1
    z_max = np.array(desired_z[0]).max()+0.1
    x_min = np.array(desired_x[0]).min()-0.1
    y_min = np.array(desired_y[0]).min()-0.1
    z_min = np.array(desired_z[0]).min()-0.1



    # Plot the first part
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(x1_ref1, y1_ref1, z1_ref1,  color='yellow', label='Reference 1', s=8, marker='o', edgecolors='black')
    # ax1.scatter(x2_ref1, y2_ref1, z2_ref1,  color='brown', label='Reference 2', s=8, marker='o')
    ax1.scatter(x1_part1_m, y1_part1_m, z1_part1_m, color='r', label='Measured 1')
    ax1.scatter(x1_part1_d, y1_part1_d, z1_part1_d, color='g', label='Desired 1')
    ax1.scatter(x2_ref1, y2_ref1, z2_ref1,  color='brown', label='Reference 2', s=8, marker='o', edgecolors='black')
    ax1.scatter(x2_part1_m, y2_part1_m, z2_part1_m, color='orange', label='Measured 2')
    ax1.scatter(x2_part1_d, y2_part1_d, z2_part1_d, color='blue', label='Desired 2')
    ax1.set_title('t = T/3')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.grid(True)
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.set_zlim(z_min, z_max)
    ax1.legend()

    # Plot the second part
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.scatter(x1_ref2, y1_ref2, z1_ref2,  color='yellow', label='Reference 1', s=8, marker='o', edgecolors='black')
    # ax2.scatter(x2_ref2, y2_ref2, z2_ref2,  color='brown', label='Reference 2', s=8, marker='o')
    ax2.scatter(x1_part2_m, y1_part2_m, z1_part2_m, color='r', label='Measured 1')
    ax2.scatter(x1_part2_d, y1_part2_d, z1_part2_d, color='g', label='Desired 1')
    ax2.scatter(x2_ref2, y2_ref2, z2_ref2,  color='brown', label='Reference 2', s=8, marker='o', edgecolors='black')
    ax2.scatter(x2_part2_m, y2_part2_m, z2_part2_m, color='orange', label='Measured 2')
    ax2.scatter(x2_part2_d, y2_part2_d, z2_part2_d, color='blue', label='Desired 2')
    ax2.set_title('t = 2T/3')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.grid(True)
    ax2.set_xlim(x_min, x_max)
    ax2.set_ylim(y_min, y_max)
    ax2.set_zlim(z_min, z_max)
    ax2.legend()

    # Plot the third part
    ax3 = fig.add_subplot(133, projection='3d')
    ax3.scatter(x1_ref3, y1_ref3, z1_ref3, color='yellow', label='Reference 1', s=8, marker='o', edgecolors='black')
    # ax3.scatter(x2_ref3, y2_ref3, z2_ref3, color='brown', label='Reference 2', s=8, marker='o')
    ax3.scatter(x1_part3_m, y1_part3_m, z1_part3_m, color='r', label='Measured 1')
    ax3.scatter(x1_part3_d, y1_part3_d, z1_part3_d, color='g', label='Desired 1')
    ax3.scatter(x2_ref3, y2_ref3, z2_ref3, color='brown', label='Reference 2', s=8, marker='o', edgecolors='black')
    ax3.scatter(x2_part3_m, y2_part3_m, z2_part3_m, color='orange', label='Measured 2')
    ax3.scatter(x2_part3_d, y2_part3_d, z2_part3_d, color='blue', label='Desired 2')
    ax3.set_title('t = T')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.grid(True)
    ax3.set_xlim(x_min, x_max)
    ax3.set_ylim(y_min, y_max)
    ax3.set_zlim(z_min, z_max)
    ax3.legend()

    # Adjust layout for better spacing
    plt.tight_layout()
    plt.show()
    pdb.set_trace()

def get_3d_plot():
    # Create a 3D figure
    fig = plt.figure(figsize=(12, 8))
    # ax = fig.add_subplot(111, projection='3d')


    x1_positions_m, y1_positions_m, z1_positions_m = measured_x[0], measured_y[0], measured_z[0]
    split_1 = len(x1_positions_m) // 3
    split_2 = 2 * len(x1_positions_m) // 3
    x1_part1_m, y1_part1_m, z1_part1_m = x1_positions_m[:split_1], y1_positions_m[:split_1], z1_positions_m[:split_1]
    x1_part2_m, y1_part2_m, z1_part2_m = x1_positions_m[:split_2], y1_positions_m[:split_2], z1_positions_m[:split_2]
    x1_part3_m, y1_part3_m, z1_part3_m = x1_positions_m, y1_positions_m, z1_positions_m

    x1_positions_d, y1_positions_d, z1_positions_d = desired_x[0], desired_y[0], desired_z[0]
    split_1 = len(x1_positions_d) // 3
    split_2 = 2 * len(x1_positions_d) // 3
    x1_part1_d, y1_part1_d, z1_part1_d = x1_positions_d[:split_1], y1_positions_d[:split_1], z1_positions_d[:split_1]
    x1_part2_d, y1_part2_d, z1_part2_d = x1_positions_d[:split_2], y1_positions_d[:split_2], z1_positions_d[:split_2]
    x1_part3_d, y1_part3_d, z1_part3_d = x1_positions_d, y1_positions_d, z1_positions_d

    x2_positions_m, y2_positions_m, z2_positions_m = measured_x[1], measured_y[1], measured_z[1]
    split_1 = len(x1_positions_m) // 3
    split_2 = 2 * len(x1_positions_m) // 3
    x2_part1_m, y2_part1_m, z2_part1_m = x2_positions_m[:split_1], y2_positions_m[:split_1], z2_positions_m[:split_1]
    x2_part2_m, y2_part2_m, z2_part2_m = x2_positions_m[:split_2], y2_positions_m[:split_2], z2_positions_m[:split_2]
    x2_part3_m, y2_part3_m, z2_part3_m = x2_positions_m, y2_positions_m, z2_positions_m

    x2_positions_d, y2_positions_d, z2_positions_d = desired_x[1], desired_y[1], desired_z[1]
    split_1 = len(x1_positions_d) // 3
    split_2 = 2 * len(x1_positions_d) // 3
    x2_part1_d, y2_part1_d, z2_part1_d = x2_positions_d[:split_1], y2_positions_d[:split_1], z2_positions_d[:split_1]
    x2_part2_d, y2_part2_d, z2_part2_d = x2_positions_d[:split_2], y2_positions_d[:split_2], z2_positions_d[:split_2]
    x2_part3_d, y2_part3_d, z2_part3_d = x2_positions_d, y2_positions_d, z2_positions_d
    
    x_max = np.array(desired_x[0]).max()+0.1
    y_max = np.array(desired_y[0]).max()+0.1
    z_max = np.array(desired_z[0]).max()+0.1
    x_min = np.array(desired_x[0]).min()-0.1
    y_min = np.array(desired_y[0]).min()-0.1
    z_min = np.array(desired_z[0]).min()-0.1



    # Plot the first part
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.scatter(x1_part1_m, y1_part1_m, z1_part1_m, color='r', label='Measured 1')
    ax1.scatter(x1_part1_d, y1_part1_d, z1_part1_d, color='g', label='Desired 1')
    ax1.scatter(x2_part1_m, y2_part1_m, z2_part1_m, color='orange', label='Measured 2')
    ax1.scatter(x2_part1_d, y2_part1_d, z2_part1_d, color='blue', label='Desired 2')
    ax1.set_title('t = T/3')
    ax1.set_xlabel('X')
    ax1.set_ylabel('Y')
    ax1.set_zlabel('Z')
    ax1.grid(True)
    ax1.set_xlim(x_min, x_max)
    ax1.set_ylim(y_min, y_max)
    ax1.set_zlim(z_min, z_max)
    ax1.legend()

    # Plot the second part
    ax2 = fig.add_subplot(132, projection='3d')
    ax2.scatter(x1_part2_m, y1_part2_m, z1_part2_m, color='r', label='Measured 1')
    ax2.scatter(x1_part2_d, y1_part2_d, z1_part2_d, color='g', label='Desired 1')
    ax2.scatter(x2_part2_m, y2_part2_m, z2_part2_m, color='orange', label='Measured 2')
    ax2.scatter(x2_part2_d, y2_part2_d, z2_part2_d, color='blue', label='Desired 2')
    ax2.set_title('t = 2T/3')
    ax2.set_xlabel('X')
    ax2.set_ylabel('Y')
    ax2.set_zlabel('Z')
    ax2.grid(True)
    ax2.set_xlim(x_min, x_max)
    ax2.set_ylim(y_min, y_max)
    ax2.set_zlim(z_min, z_max)
    ax2.legend()

    # Plot the third part
    ax3 = fig.add_subplot(133, projection='3d')
    ax3.scatter(x1_part3_m, y1_part3_m, z1_part3_m, color='r', label='Measured 1')
    ax3.scatter(x1_part3_d, y1_part3_d, z1_part3_d, color='g', label='Desired 1')
    ax3.scatter(x2_part3_m, y2_part3_m, z2_part3_m, color='orange', label='Measured 2')
    ax3.scatter(x2_part3_d, y2_part3_d, z2_part3_d, color='blue', label='Desired 2')
    ax3.set_title('t = T')
    ax3.set_xlabel('X')
    ax3.set_ylabel('Y')
    ax3.set_zlabel('Z')
    ax3.grid(True)
    ax3.set_xlim(x_min, x_max)
    ax3.set_ylim(y_min, y_max)
    ax3.set_zlim(z_min, z_max)
    ax3.legend()

    # Adjust layout for better spacing
    plt.tight_layout()
    plt.show()
    pdb.set_trace()

if __name__=="__main__":
    pb.connect(pb.GUI)
    # pb.connect(pb.DIRECT)
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

    robot1.initialize(base_pos=[0.0, 0.2, 0.0], base_ori=[0,0,0,1])
    robot2.initialize(base_pos=[0.65, 0.2, 0.0], base_ori=[0,0,0.707, 0.707])


    pb.setTimeStep(1/100)


    # Trajectory 2
    # Trajectory

    t_des = np.linspace(0.0, 1.0, 1000)

    '''
    Line Trajectory
    '''
    start1 = robot1._getLinkState(robot1.end_effector_index)[0]
    end2 = [0.0, 0.5, 0.2]
    start2 = robot2._getLinkState(robot2.end_effector_index)[0]
    end1 = [0.5, 0.5, 0.2]

    print('start1',start1)
    print('start2',start2)

    gamma = np.array([np.linspace(start1[i], end1[i], len(t_des)) for i in range(3)]).T
    gamma2 = np.array([np.linspace(start2[i], end2[i], len(t_des)) for i in range(3)]).T

    print(gamma)
    print(gamma2)

    # pdb.set_trace()

    '''
    TODO: Make more examples of trajectories.
    '''

    alpha1,T1,R1=tra_map_param(gamma)
    gamma_map=tra_map_forward(gamma, alpha1, T1, R1)

    alpha2,T2,R2=tra_map_param(gamma2)
    gamma2_map=tra_map_forward(gamma2, alpha2, T2, R2)


    # call the function to obtain data_dict_agent1
    
    # cent_agent1, axes_agent1,velo_agent1,rotm_agent1 = getLinkFeatures(robot1)
    # cent_agent2, axes_agent2,velo_agent2,rotm_agent2 = getLinkFeatures(robot2)
    # Update_link_Data(cent_agent1, cent_agent2)

    # plot3d_(data_dict_agent1['link3'])
    
    

    # Create a 3D plot
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

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
    tol=0.01
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

    print(gamma_[0],gamma_[-1])
    print(gamma2_[0],gamma2_[-1])
    # pdb.set_trace()

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

    # dyn_lambda = 6.74
    # dyn_lambda = 100
    # dyn_eta = 1.14
    # dyn_beta = 0.55
    Potential_Params=[0.9220,0.2862]
    A = Potential_Params[0]
    eta = Potential_Params[1]
    # dyn_beta = Potential_Params[2]

    #contact save axes
    axes_safe=np.array([0.15,0.15,0.15])

    cent_agent1, axes_agent1,velo_agent1,rotm_agent1 = getLinkFeatures(robot1)
    cent_agent2, axes_agent2,velo_agent2,rotm_agent2 = getLinkFeatures(robot2)
    
    for i in range(1, num_links+1):
        center_map=cent_agent1[f"link{i}"]
        center_map=tra_map_forward(center_map, alpha2, T2, R2)
        axes_map=axes_agent1[f"link{i}"]
        axes_map= alpha2 * axes_map
        Ellipsoid_dict_agent1[f"link{i}"]=sq_stat(center=center_map, axis=axes_map+axes_safe, coeffs=np.array([1,1,1]), A=A, eta=eta)

        center_map=cent_agent2[f"link{i}"]
        center_map=tra_map_forward(center_map, alpha1, T1, R1)
        axes_map=axes_agent1[f"link{i}"]
        axes_map=alpha1 * axes_map
        Ellipsoid_dict_agent2[f"link{i}"]=sq_stat(center=center_map, axis=axes_map+axes_safe, coeffs=np.array([1,1,1]), A=A, eta=eta)

    Update_link_Data(cent_agent1, cent_agent2)

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
    with imageio.get_writer('Kinematic_DMP_cross3.gif', mode='I', fps=10) as writer:
        while total_time < 0.8:

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
                
                
                # pdb.set_trace()
                if not close_flag:
                    print("agent1 stop")

                if (not flag_conv) and close_flag:
                    # apply potential field
                    print('ready to update agent1')
                    MP.step(external_force=perturb_dynamic_agent1)
                    # flag_conv = np.linalg.norm(MP.x - MP.x_goal) < MP.tol
                    print('agent1 update')
                    
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
                    flag_conv = np.linalg.norm(meas1 - MP.x_goal) < 0.05
                
                    print(np.linalg.norm(meas1- MP.x_goal))

                    # call the function to obtain data_dict_agent1

                    cent_agent1, axes_agent1,velo_agent1,rotm_agent1 = getLinkFeatures(robot1)
                    cent_agent2, axes_agent2,velo_agent2,rotm_agent2 = getLinkFeatures(robot2)
                    Update_link_Data(cent_agent1, cent_agent2)

                    # plot3d_2(data_dict_agent1[key])

                    for i in range(1,num_links+1):
                        key = f'link{i}'
                        # print('Vel-->', key, velo_agent1[key])
                    #     points_ = tra_map_forward(data_dict_agent1[key],alpha2,T2,R2)

                    #     plot_ellipsoid_and_points(Center_dict_agent1[key], Axes_dict_agent1[key], points_, ax)
                    #     plot3d_(points_)
                        # plot3d_2(data_dict_agent1[key])
                        # plot3d_2(gamma)
                        
                        # pt = cent_agent1[key]
                        # ax.plot(pt[0],pt[1],pt[2], marker="o", color="r",markersize=10)

                        # plt.show()
                        # pdb.set_trace() 



                

                if not flag_conv2:

                    if total_time == 0.03:
                        pdb.set_trace()
                    elif total_time >= 0.3 and total_time <= 0.32:
                        pdb.set_trace()
                    elif total_time >= 0.6 and total_time <= 0.62:
                        pdb.set_trace()
                    # apply potential field
                    print('ready to update agent2')
                    MP2.step(external_force=perturb_dynamic_agent2)
                    print('agent2 update')
                    # flag_conv2 = np.linalg.norm(MP2.x - MP2.x_goal) < MP2.tol
                    # Map coordinations
                    center_agent2_origin=tra_map_inverse(np.array(MP2.x), alpha2, T2, R2)

                    #store trajecory in world map
                    gamma2_tra = np.append(gamma2_tra, np.array([center_agent2_origin]), axis=0)
                    position_ee = gamma2_tra[-1,:]
                    rotation_ee = [0,0,0.707,0.707]
                    # rotation_ee = [0.707,0,0,0.707]
                    joint_angle = robot2._calculateIK(position_ee,rotation_ee)
                    # robot2._sendPositionCommand(joint_angle)
                    robot2._resetJointState(joint_angle)
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
                    flag_conv2 = np.linalg.norm(meas2- MP2.x_goal) < 0.05
                    print(np.linalg.norm(meas2- MP2.x_goal))

                    # call the function to obtain data_dict_agent1

                    cent_agent1, axes_agent1,velo_agent1,rotm_agent1 = getLinkFeatures(robot1)
                    cent_agent2, axes_agent2,velo_agent2,rotm_agent2 = getLinkFeatures(robot2)
                    Update_link_Data(cent_agent1, cent_agent2)

                time.sleep(0.01)
                print('--------------------->', total_time)

                if flag_conv:
                    print("agent1 achieve goal")

                if flag_conv2:
                    print("agent2 achieve goal")

                
                # plotresult(writer)

    get_3d_plot2()

    # Create a 3D plot
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # # Plot measured values
    # # Plot measured values
    # ax.scatter(measured_x[0], measured_y[0], measured_z[0], color='blue', label='Measured 1', s=10)

    # # Plot desired values
    # ax.scatter(desired_x[0], desired_y[0], desired_z[0], color='red', label='Desired 1', s=10)


    # # Plot measured values
    # ax.scatter(measured_x[1], measured_y[1], measured_z[1], color='orange', label='Measured 2', s=10)

    # # Plot desired values
    # ax.scatter(desired_x[1], desired_y[1], desired_z[1], color='green', label='Desired 2', s=10)

    # # Add labels
    # ax.set_xlabel('X')
    # ax.set_ylabel('Y')
    # ax.set_zlabel('Z')


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
    # ax.legend()

    # # Show plot
    # plt.show()
    # print(total_time)
    # pdb.set_trace()
