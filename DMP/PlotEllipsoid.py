

def plot_ellipsoid(ax, center, axes_lengths):
    u = np.linspace(0, 2 * np.pi, 100)
    v = np.linspace(0, np.pi, 100)
    x = axes_lengths[0] * np.outer(np.cos(u), np.sin(v)) + center[0]
    y = axes_lengths[1] * np.outer(np.sin(u), np.sin(v)) + center[1]
    z = axes_lengths[2] * np.outer(np.ones_like(u), np.cos(v)) + center[2]

    ax.plot_surface(x, y, z, color='b', alpha=0.3)

# Create a 3D plot
fig = plt.figure()
ax = fig.add_subplot(111, projection='3d')

# Plot each ellipsoid
for ellipsoid_name in centers:
    center = centers[ellipsoid_name]
    axes_lengths = axes[ellipsoid_name]
    plot_ellipsoid(ax, center, axes_lengths)

# Set labels
ax.set_xlabel('X')
ax.set_ylabel('Y')
ax.set_zlabel('Z')

# Optionally set equal aspect ratio (same as the previous response)
max_range = np.array([max([axes_lengths[0] for axes_lengths in axes.values()]),
                      max([axes_lengths[1] for axes_lengths in axes.values()]),
                      max([axes_lengths[2] for axes_lengths in axes.values()])]).max()

mid_x = np.mean([center[0] for center in centers.values()])
mid_y = np.mean([center[1] for center in centers.values()])
mid_z = np.mean([center[2] for center in centers.values()])

ax.set_xlim(mid_x - max_range, mid_x + max_range)
ax.set_ylim(mid_y - max_range, mid_y + max_range)
ax.set_zlim(mid_z - max_range, mid_z + max_range)

# Show the plot
plt.show()