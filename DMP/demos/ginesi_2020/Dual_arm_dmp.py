import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from io import BytesIO
import imageio

from dmp.dmp import DMPs_cartesian
from dmp.obstacle_superquadric import Obstacle_Dynamic as sq_dyn



def perturb_dynamic_agent1(x, v):
    out = obst_dynamic.gen_external_force(x, v-v_e)+\
    agent1_dynamic.gen_external_force(x,v-v_agent2)
    # print(out)
    return out

def perturb_dynamic_agent2(x, v):
    out = obst_dynamic.gen_external_force(x, v-v_e)+\
    agent2_dynamic.gen_external_force(x,v-v_agent1)
    return out

#DMP 3D plot
def plotresult(writer):
    # with imageio.get_writer('animated_plot.gif', mode='I', duration=0.2) as writer:
    fig=plt.figure(figsize=(10, 10))
    ax = fig.add_subplot(111, projection='3d')
    ax.plot(gamma_[:, 0], gamma_[:, 1], gamma_[:, 2],'--k', linewidth=1.5)
    ax.plot(gamma2_[:, 0], gamma2_[:, 1], gamma2_[:, 2],'--k', linewidth=1.5)
    # ax.plot(gamma[:, 0], gamma[:, 1], gamma[:, 2],'-g', linewidth=1.5)
    ax.plot(gamma_tra[:, 0], gamma_tra[:, 1], gamma_tra[:, 2],'-r', linewidth=1.5)
    ax.plot(gamma2_tra[:, 0], gamma2_tra[:, 1], gamma2_tra[:, 2],'-r', linewidth=1.5)
    ax.plot_surface(x, y, z, color='b', alpha=0.3, edgecolor=(0, 0, 0, 0.1))  # Use alpha for transparency
    ax.plot_surface(x_, y_, z_, color='r', alpha=0.3, edgecolor=(0, 0, 0, 0.1))  # Use alpha for transparency
        
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

if __name__=="__main__":
    # Trajectory 2
    # Trajectory
    t_des = np.linspace(0.0, 1.0, 1000)
    gamma = np.transpose(
        [np.cos(np.pi * t_des),
        np.sin(np.pi * t_des),
        t_des * 0.5])

    gamma2 = np.transpose(
        [np.cos(np.pi * t_des),
        1-np.sin(np.pi * t_des),
        t_des * 0.5])

    #DMP Ini
    K=1050.0
    alpha_s = 4.0
    tol=0.01
    dt=0.01

    MP = DMPs_cartesian(K=K, alpha_s=alpha_s, tol=tol)
    MP2= DMPs_cartesian(K=K, alpha_s=alpha_s, tol=tol)

    #DMP try
    MP.imitate_path(t_des=t_des, x_des=gamma)
    MP2.imitate_path(t_des=t_des, x_des=gamma2)

    gamma_, _, _, _ = MP.rollout()
    gamma2_, _, _, _ = MP2.rollout()
    t_des_ = np.linspace(0.0, 1.0, np.shape(gamma)[0])


    # Plot Ellipsoid
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
    gamma_tra = np.array([MP.x_0])
    gamma2_tra= np.array([MP2.x_0])

    dyn_lambda = 10.0
    dyn_eta = 1.0
    dyn_beta = 1.0/2

    # obstacle
    center_e=np.array([0.0,1.0,0.25])
    axes_e=np.array([a,b,c])
    v_e = np.array([0.0,0.2,0.0])

    #agent
    center_agent1=np.array(MP.x_0)
    center_agent2=np.array(MP2.x_0)

    axes_agent=np.array([0.1,0.1,0.1])

    v_agent1=np.array([0.0,0.0,0.0])
    v_agent2=np.array([0.0,0.0,0.0])


    obst_dynamic = sq_dyn(center=center_e, axis=axes_e, coeffs=np.array([1,1,2]), lmbda=dyn_lambda, beta=dyn_beta, eta=dyn_eta)
    agent1_dynamic= sq_dyn(center=center_agent2, axis=axes_agent, coeffs=np.array([1,1,1]), lmbda=dyn_lambda, beta=dyn_beta, eta=dyn_eta)
    agent2_dynamic= sq_dyn(center=center_agent1, axis=axes_agent, coeffs=np.array([1,1,1]), lmbda=dyn_lambda, beta=dyn_beta, eta=dyn_eta)

    plt.ion()

    flag_conv = False
    flag_conv2 = False
    total_time=0

    # while not (flag_conv and flag_conv2):
    with imageio.get_writer('animated_plot.gif', mode='I', duration=0.2) as writer:
        while not (flag_conv and flag_conv2):
            obst_dynamic.center=obst_dynamic.center+v_e*dt
            total_time += dt
            
            close_flag= np.linalg.norm(MP.x - MP2.x)>0.2
            # close_flag=True

                # Parametric equations for the ellipsoid
            x_ = a * np.sin(phi) * np.cos(theta) + obst_dynamic.center[0]
            y_ = b * np.sin(phi) * np.sin(theta) + obst_dynamic.center[1]
            z_ = c * np.cos(phi) + obst_dynamic.center[2]

            if not close_flag:
                print("agent1 stop")

            if (not flag_conv) and close_flag:
                MP.step(external_force=perturb_dynamic_agent1)
                gamma_tra = np.append(gamma_tra, np.array([MP.x]), axis=0)
                flag_conv = np.linalg.norm(MP.x - MP.x_goal) < MP.tol
                agent2_dynamic.center=np.array(MP.x)
                v_agent1=np.array(MP.dx)
            

            if not flag_conv2:
                MP2.step(external_force=perturb_dynamic_agent2)
                gamma2_tra = np.append(gamma2_tra, np.array([MP2.x]), axis=0)
                flag_conv2 = np.linalg.norm(MP2.x - MP2.x_goal) < MP2.tol
                agent1_dynamic.center=np.array(MP2.x)
                v_agent2=np.array(MP2.dx)


            if flag_conv:
                print("agent1 achieve goal")

            if flag_conv2:
                print("agent2 achieve goal")
            
            # plotresult(writer=writer)


    print(total_time)