# Dual Arm Cooperative DMP

## Overview

We introduce a  new method that utilizes collaborative DMP approach to generate stable, collision-free trajectories for multiple arms. Called ONCol-DMP, the proposed method combines RL with DMP, leveraging human demonstrations to generate online trajectories in real-time. Our paper is currently under review. 

[Follow this link for the project website.](https://sites.google.com/virginia.edu/oncoldmp/home)

## Prerequisites

Before proceeding with the installation, ensure you have the following installed:

- conda         23.9
- python        3.9
- matplotlib    3.8.2
- numpy         1.26.0
- pybullet      3.2.5
- pyaml         23.9.7

## Installation

1. **Create a new folder**  
   Open a terminal and navigate to the directory where you'd like to create your project folder. Then, run the following command:

   ```bash
   mkdir <your-project-name>
   cd <your-project-name>
   ```

2. **Create a new `conda` env**
   Create a new virtual env. We suggest using conda.

   ```bash
   conda create --name dual_arm python=3.9
   conda activate dual_arm
   ```
   *Note*: You might have to install some other dependencies as well, such as ```matplotlib, scipy, numpy``` etc. We expect that you install those as prompted by your system.

3. **Intall PyBullet**
   You can build PyBullet from source if you wish. We suggest installing usinig `pip`:

   ```bash
   pip install pybullet
   ```
4. **Install BulletArm**
   ```bash
   git clone https://github.com/ColinKohler/BulletArm.git
   cd BulletArm
   ```
   Install dependencies:
   ```bash
   pip install -r requirements.txt 
   ```
   Install this package:
   ```bash
   cd ../
   pip install BulletArm
   ```
   Or add to your PYTHONPATH
   ```bash
   export PYTHONPATH=/path/to/BulletArm/:$PYTHONPATH
   ```

5. **Unzip the `Dual_Arm` file**  
   After cloning the repository, unzip the `Dual_Arm.zip` file by running:

   ```bash
   unzip Dual_Arm.zip
   ```

6. **Install DMP**
    Install DMP in `Dual_Arm` project folder:

    ```bash
    pip install DMP
    ```

7. **Start Editing the DMP File**
   The main file is the `DualArm_DMP_PyB.py`. Make sure that this file is in the `DMP/` folder. If placing this any deeper then you will have to change the `import` to import the `Robot` class objects from `Robot_Sim` directly, but that can have issues.

## License
Code licensed under the Apache License v2.0
