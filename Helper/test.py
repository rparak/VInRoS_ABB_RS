# System (Default)
import sys
#   Add access if it is not in the system path.
if '../../' + 'src' not in sys.path:
    sys.path.append('../../' + 'src')
# Numpy (Array computing) [pip3 install numpy]
import numpy as np
# Time (Time access and conversions)
import time
# OS (Operating system interfaces)
import os
# Custom Lib.:
#   ../Lib/EGM/Core
import Lib.EGM.Core
#   ../Lib/Transsformation/Utilities/Mathematics
import Lib.Transformation.Utilities.Mathematics as Mathematics
#   ../Lib/Trajectory/Utilities
import Lib.Trajectory.Utilities
#   ../Lib/Utilities/File_IO
import Lib.Utilities.File_IO as File_IO

"""
Description:
    Initialization of constants.
"""
# EGM time step.
CONST_DT = 0.004
# Simulation stop(t_0), start(t_1) time in seconds.
CONST_T_0 = 0.0
CONST_T_1 = 5.0

def main():
    """
    Description:
        A program to collect data obtained from the robot via EGM.

        Note 1:
            The data will be used for the analysis of the precision between the actual 
            and desired positions of the robot's joints.

        Note 2:
            The selected robot is the ABB IRB 120, a six-axis articulated robot.
    """
    
    theta_desired = []; theta_actual = []
    
    # Locate the path to the project folder.
    project_folder = os.getcwd().split('ABB_EGM_Python')[0] + 'ABB_EGM_Python'

    # Initialization of the class.
    #   Start UDP communication.
    ABB_IRB_120_EGM_Cls = Lib.EGM.Core.EGM_Control_Cls('192.168.0.23', 6512)

    ABB_IRB_120_EGM_Cls.Set_Absolute_Joint_Position(np.array([0.0] * 7, dtype=np.float64), True)

    print(ABB_IRB_120_EGM_Cls.Theta)
    
if __name__ == '__main__':
    sys.exit(main())