import numpy as np
from sympy.plotting.textplot import linspace


def rotation_matrix_from_vectors(v1, v2):
    # v2= R @ v1

    v1 = v1 / np.linalg.norm(v1)
    v2 = v2 / np.linalg.norm(v2)
    axis = np.cross(v1, v2)
    # print(axis)
    axis_norm = np.linalg.norm(axis)
    cos_theta = np.dot(v1, v2)
    theta = np.arccos(np.clip(cos_theta, -1.0, 1.0))

    if axis_norm == 0:
        if cos_theta > 0:
            return np.eye(3)  # No rotation needed
        else:
            # Find an orthogonal vector to construct 180-degree rotation
            orthogonal_vector = np.array([1, 0, 0])
            axis = np.cross(v1, orthogonal_vector)

            if np.linalg.norm(axis) ==0:
                orthogonal_vector=np.array([0, 1, 0])
                axis = np.cross(v1, orthogonal_vector)

            # axis_norm = np.linalg.norm(axis)
            axis = axis / np.linalg.norm(axis)
            theta = np.pi


    axis = axis / np.linalg.norm(axis)
    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])
    R = np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)

    return R


def tra_map_param(tra):
    tra=np.array(tra)
    T=tra[0]

    v1 = tra[-1] - tra[0]
    v2=np.array([1,1,1])


    alpha=np.linalg.norm(v2) / np.linalg.norm(v1)

    v1_n=v1/np.linalg.norm(v1)
    v2_n=v2/np.linalg.norm(v2)

    R=rotation_matrix_from_vectors(v1_n, v2_n)

    tra_new=tra_map_forward(tra, alpha, T, R)

    distances = [distance_from_line(point, np.array([0, 0, 0]), np.array([1,1,1])) for point in tra_new]
    max_distance_index = np.argmax(distances)

    axis=np.array([1,1,1])
    axis=axis/np.linalg.norm(axis)

    K = np.array([
        [0, -axis[2], axis[1]],
        [axis[2], 0, -axis[0]],
        [-axis[1], axis[0], 0]
    ])

    # while not (np.all(tra_new>=-1e4)):
    R_addition_=np.eye(3)

    max_num= np.sum(tra_new>=-1e-3)
    max_point=np.sum(tra_new[max_distance_index][2])
    # print(max_num)

    for theta in linspace(0, 2*np.pi,500):

        # if np.all(tra_new>=-1e-3):
        #     print("all non-neg")
        #     break
        # else:
        R_addition=np.eye(3) + np.sin(theta) * K + (1 - np.cos(theta)) * np.dot(K, K)
        tra_new = tra_map_forward(tra, alpha, T, R_addition @ R)
        # print("pos num",pos_num)
        # print("rotate angle",theta / np.pi * 180)

        if np.sum(tra_new>=-1e-3)>=max_num:
            max_num=np.sum(tra_new>=-1e-3)
            if np.sum(tra_new[max_distance_index][2])>=max_point:
                max_point=np.sum(tra_new[max_distance_index][2])
                R_addition_=R_addition
            # print(max_num)
            # print("rotate angle",theta / np.pi * 180)

            # print(tra_new)

    R=R_addition_ @ R
    return alpha, T, R


def distance_from_line(point, line_point, line_dir):
    # Vector from the line point to the point
    point_vector = point - line_point
    # Cross product of point vector and line direction vector
    cross_prod = np.cross(point_vector, line_dir)
    # Norm of the cross product
    cross_prod_norm = np.linalg.norm(cross_prod)
    # Norm of the line direction vector
    line_dir_norm = np.linalg.norm(line_dir)
    # Distance calculation
    distance = cross_prod_norm / line_dir_norm
    return distance


def tra_map_forward(tra, alpha, T, R):
    tra_new=alpha * R @ (tra - T).T
    tra_new=tra_new.T
    return tra_new

def tra_map_inverse(tra, alpha, T, R):
    tra_new= R.T @ (tra/alpha).T
    tra_new = tra_new.T + T
    return tra_new



# Example usage:
# v1 = np.array([0,0,1])
# v2 = np.array([0,0,-1])
# R_matrix = rotation_matrix_from_vectors(v1, v2)

# v3=R_matrix @ v1
# print("Rotation Matrix R:\n", R_matrix)
# print("inverse map",v3)


# tra=np.array([[0,0,0],[1.2,1,0],[1,0,0],[2,3,3]])
# alpha,T,R=tra_map_param(tra)
# tra_new=tra_map_forward(tra, alpha, T, R)
# tra_origin=tra_map_inverse(tra_new,alpha, T, R)
# tra_origin_element=tra_map_inverse(tra_new[1],alpha, T, R)

# print("tra_new:", tra_new)
# print("tra_origin",tra_origin)
# print("tra_origin_element",tra_origin_element)