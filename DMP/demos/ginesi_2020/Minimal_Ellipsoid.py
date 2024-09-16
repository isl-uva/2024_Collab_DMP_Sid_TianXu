import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from scipy.optimize import minimize
from DMP_map_Rotation import  tra_map_forward


def ellipsoid_volume(x):
    # Objective function: minimize the logarithm of the volume
    a, b, c = x[:3]
    return np.log(a) + np.log(b) + np.log(c)  # Minimize the sum of logs of the axes lengths

def ellipsoid_constraint(x, points):
    # Constraints ensuring all points are within the ellipsoid
    a, b, c, cx, cy, cz = x
    return [1 - ((p[0] - cx)**2 / a**2 + (p[1] - cy)**2 / b**2 + (p[2] - cz)**2 / c**2) for p in points]

def find_minimum_volume_ellipsoid(points):
    # Initial guess for a, b, c, and center coordinates
    # Start with 1 for axes and mean of points for the center
    means = np.mean(points, axis=0)
    initial_guess = [1, 1, 1] + means.tolist()

    # Bounds to ensure positive radii and free center coordinates
    bounds = [(0.01, None), (0.01, None), (0.01, None)] + [(None, None)] * 3

    # Construct the constraints in a form acceptable by scipy.optimize.minimize
    cons = {'type': 'ineq', 'fun': ellipsoid_constraint, 'args': (points,)}

    # Optimization
    result = minimize(ellipsoid_volume, initial_guess, method='SLSQP', constraints=cons, bounds=bounds)

    if result.success:
        a, b, c, cx, cy, cz = result.x
        return (cx, cy, cz), (a, b, c), result.fun
    else:
        raise ValueError("Optimization failed: " + result.message)


def plot_ellipsoid_and_points(center, axes, points, ax):
    """
    Plots an ellipsoid given center and axes along with a set of points.

    Parameters:
    center (tuple): The (x, y, z) coordinates of the ellipsoid center.
    axes (tuple): The radii of the ellipsoid (a, b, c) along the x, y, z axes.
    points (np.array): An Nx3 array of points to plot.
    """
    # fig = plt.figure()
    # ax = fig.add_subplot(111, projection='3d')

    # Ellipsoid
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = axes[0] * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = axes[1] * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = axes[2] * np.outer(np.ones(np.size(u)), np.cos(v)) + center[2]
    ax.plot_surface(x, y, z, color='b', alpha=0.6)

    # Points
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], color='r')

    # Setting labels
    ax.set_xlabel('X axis')
    ax.set_ylabel('Y axis')
    ax.set_zlabel('Z axis')
    ax.set_title('Ellipsoid and Points')

    # Show the plot
    # plt.show()
def draw_ellipsoid(ax, center, axes_lengths, num_points=100, color='b', alpha=0.5):
    """
    Draw an ellipsoid in an existing 3D plot.

    Parameters:
        ax: The matplotlib 3D axis where the ellipsoid will be plotted.
        center: A tuple or array-like containing the (x, y, z) coordinates of the ellipsoid center.
        axes_lengths: A tuple or array-like containing the lengths of the ellipsoid axes (a, b, c).
        num_points: The number of points to use for creating the ellipsoid surface (default is 100).
        color: The color of the ellipsoid (default is blue).
        alpha: The transparency level of the ellipsoid (default is 0.5).
    """
    # Unpack the center and axes lengths
    cx, cy, cz = center
    a, b, c = axes_lengths

    # Create the mesh grid
    u = np.linspace(0, 2 * np.pi, num_points)
    v = np.linspace(0, np.pi, num_points)
    u, v = np.meshgrid(u, v)

    # Parametric equations for the ellipsoid surface
    x = a * np.cos(u) * np.sin(v) + cx
    y = b * np.sin(u) * np.sin(v) + cy
    z = c * np.cos(v) + cz

    # Plot the ellipsoid
    ax.plot_surface(x, y, z, color=color, alpha=alpha, rstride=4, cstride=4)

def rotate_ellipsoid(center, axes, alpha,T, R):
    axes_matrix=np.array([[axes[0],0,0],
                          [-axes[0],0,0],
                          [ 0,axes[1], 0],
                          [ 0, -axes[1],0],
                          [ 0, 0,axes[2]],
                          [0, 0,-axes[2]],
                          ])

    points=center+axes_matrix
    points_=tra_map_forward(points,alpha,T,R)

    center_new, axes_new, _ = find_minimum_volume_ellipsoid(points_)
    return center_new, axes_new, points_


# # Example usage
# points = np.array([
#     [0, 0, 0],
#     [4, 5, 6],
#     [7, 8, 9],
#     [10, 11, 12]
# ])
#
# center, axes, volume = find_minimum_volume_ellipsoid(points)
# print("Ellipsoid Center:", center)
# print("Ellipsoid Axes:", axes)
# print("Volume of Ellipsoid:", np.exp(volume))  # Exp to convert from log volume
#
# fig = plt.figure()
# ax = fig.add_subplot(111, projection='3d')
# plot_ellipsoid_and_points(center, axes, points,ax)