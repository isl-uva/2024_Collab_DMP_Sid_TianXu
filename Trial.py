import matplotlib.pyplot as plt
import seaborn as sns
import numpy as np
import pandas as pd

# Block = 0.25
matrix1 = np.array([[0.14, 0.16, 0.15, 0.08],  # v = 0.05
                    [0.14, 0.16, 0.15, 0.08],  # v = 0.10
                    [0.14, 0.16, 0.15, 0.08],  # v = 0.15
                    [0.14, 0.16, 0.15, 0.08]]) # v =  0.2
# Block = 0.50
matrix2 = np.array([[0.27, 0.35, 0.23, 0.17],  # v = 0.05
                    [0.27, 0.35, 0.23, 0.17],# v = 0.10
                    [0.27, 0.35, 0.23, 0.17],# v = 0.15
                    [0.27, 0.35, 0.23, 0.17]]) # v =  0.2
# Block = 0.75
matrix3 = np.array([[0.38, 0.42, 0.40, 0.2],  # v = 0.05
                    [0.38, 0.42, 0.40, 0.2],# v = 0.10
                    [0.38, 0.42, 0.40, 0.2],# v = 0.15
                    [0.38, 0.42, 0.40, 0.2]]) # v =  0.2

# # Block = 0.25
# matrix1 = np.array([[0.14, 0.16, 0.15, 0.08]]) # v =  0.2
# # Block = 0.50
# matrix2 = np.array([[0.27, 0.35, 0.23, 0.17]]) # v =  0.2
# # Block = 0.75
# matrix3 = np.array([[0.38, 0.42, 0.40, 0.2]]) # v =  0.2

# Convert each matrix to a DataFrame
df1 = pd.DataFrame(matrix1, columns=[f'v = {0.05*(i+1):.2f}' for i in range(4)])
df2 = pd.DataFrame(matrix2, columns=[f'v = {0.05*(i+1):.2f}' for i in range(4)])
df3 = pd.DataFrame(matrix3, columns=[f'v = {0.05*(i+1):.2f}' for i in range(4)])


# Set up the plot
plt.figure(figsize=(10, 6))

# Plot the boxplots for each matrix with custom colors
sns.boxplot(data=df1, width=0.2, color='#9467bd', fliersize=0)  # Purple for matrix 1
sns.boxplot(data=df2, width=0.2, color='#e377c2', fliersize=0)  # Pink for matrix 2
sns.boxplot(data=df3, width=0.2, color='#17becf', fliersize=0)  # Teal for matrix 3

# Calculate and plot the means, connecting them for each matrix with custom labels
means1 = df1.mean()
means2 = df2.mean()
means3 = df3.mean()

sns.pointplot(data=means1, color='#9467bd', markers='o', linestyles='-', scale=1.5, label='a = 0.20m')  # Purple
sns.pointplot(data=means2, color='#e377c2', markers='o', linestyles='-', scale=1.5, label='a = 0.50m')   # Pink
sns.pointplot(data=means3, color='#17becf', markers='o', linestyles='-', scale=1.5, label='a = 0.70m')  # Teal

# Add title, labels, and a custom legend
plt.title('Maximum Deviation for 3 Block Sizes', fontsize=16)
plt.xlabel('Speed (m/s)', fontsize=16)
plt.ylabel('Max. Deviation (m)', fontsize=16)

# Add the legend with custom labels
plt.legend(title="Block Size", fontsize=12)
plt.grid(which='both')
# Show the plot
plt.show()
