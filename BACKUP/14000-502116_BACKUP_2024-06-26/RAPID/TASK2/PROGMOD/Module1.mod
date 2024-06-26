MODULE Module1
    ! ## =========================================================================== ## 
    ! MIT License
    ! Copyright (c) 2021 Roman Parak
    ! Permission is hereby granted, free of charge, to any person obtaining a copy
    ! of this software and associated documentation files (the "Software"), to deal
    ! in the Software without restriction, including without limitation the rights
    ! to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    ! copies of the Software, and to permit persons to whom the Software is
    ! furnished to do so, subject to the following conditions:
    ! The above copyright notice and this permission notice shall be included in all
    ! copies or substantial portions of the Software.
    ! THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    ! IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    ! FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    ! AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    ! LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    ! OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    ! SOFTWARE.
    ! ## =========================================================================== ## 
    ! Author   : Roman Parak
    ! Email    : Roman.Parak@outlook.com
    ! Github   : https://github.com/rparak
    ! File Name: T_ROB_MAIN_CTRL/Module1.mod
    ! ## =========================================================================== ## 
    
    ! Description: !
    !   The main command structure for robot control !
    RECORD robot_command_main_str
        num start;
        num stop;
        num id;
        num update;
    ENDRECORD
    
    ! Description: !
    !   The main structure of the robot control parameters !
    RECORD robot_param_main_str
        num Trajectory_Size;
    ENDRECORD
    
    ! Description: !
    !   The main structure of robot status information !
    RECORD robot_status_str
        num in_position; ! Signaling when the robot is in the target position
        num active;      ! Signaling when everything on the robot's side is fine
        num id;          ! Identification number of the current state of the robot
        num update_done; ! Signaling when the parameter update is completed successfully
        num actual_trajectory_id;
    ENDRECORD
    
    ! Description: !
    !   The main structure of PLC status information !
    RECORD plc_status_str
        num active; ! Signaling when everything on the PC's (PLC) side is OK
        num update_done;
    ENDRECORD
    
    ! Description: !
    !   The main structure of status information !
    RECORD status_main_str
        plc_status_str plc;
        robot_status_str robot;
    ENDRECORD
    
    ! Description: !
    !   The main structure of internal values !
    RECORD internal_main_str
        num actual_state;               ! Current state in the main state machine
        num previous_state;             ! Previous state in the main state machine
        num accuracy_factor;            ! Accuracy factor for value conversion (Default)
        num accuracy_factor_QUATERNION; ! Accuracy factor for value conversion (QUATERNION)
        num actual_trajectory_id;
    ENDRECORD
    
    ! Description: !
    !   The main control structure of the robot !
    RECORD robot_ctrl_str
        robot_param_main_str parameter; ! Robot control parameters
        robot_command_main_str command; ! Command structure for robot control
        status_main_str status;         ! Robot status information
        internal_main_str internal;     ! Internal values
    ENDRECORD
    
    ! Call Main Structure
    PERS robot_ctrl_str rm_L_str;
    
    ! Joint Position Data
    PERS jointtarget L_J_Position{100};
    
    ! TCP Position Data
    PERS robtarget L_TCP_Position{100};

    ! Joint Speed Data
    PERS speeddata L_Speed{100};
    
    ! Joint Zone Data
    PERS zonedata L_Zone{100};

    ! Characteristics of a robotic tool (end-effector / gripper)
    PERS tooldata rob_L_Tool;
  
    VAR num aux_trajectory_index_var;
        
    ! Home Position
    ! MoveAbsJ [[0,-130,30,0,40,0], [135,9E9,9E9,9E9,9E9,9E9]] \NoEOffs, v100, fine, tool0;
    
    ! Description:               !
    !   Program Main Cycle:      !
    !       Type        : Normal !
    !       TrustLeve   : N/A    !
    !       Motion Task : YES    !
    PROC Main()
        TEST L_ST_MAIN_ID_PROFINET_OUT
            CASE 10: ! INITIALIZATION STATE !
                ! Description: !
                !   Initialization state to check everything on the PC's (PLC) side and activation to control !
                
                SetDO L_ST_IN_POS_PROFINET_OUT, 1;
                
                IF L_PLC_ST_ACTIVE_PROFINET_IN = 1 THEN
                    ! Wait State. Everything on the PC's (PLC) side is OK
                    SetGO L_ST_MAIN_ID_PROFINET_OUT, 20;
                ENDIF
                
            CASE 20: ! WAIT STATE !
                ! Description: !
                !   State of waiting for command with index from the PC's (PLC) !

                aux_trajectory_index_var := 0;
                
                IF L_PLC_CMD_START_PROFINET_IN = 1 AND L_PLC_CMD_MAIN_ID_PROFINET_IN = 1 THEN
                    SetDO L_ST_IN_POS_PROFINET_OUT, 0;
                    ! Move Joint (Absolute)
                    SetGO L_ST_MAIN_ID_PROFINET_OUT, 30;
                ELSEIF L_PLC_CMD_START_PROFINET_IN = 1 AND L_PLC_CMD_MAIN_ID_PROFINET_IN = 2 THEN
                    SetDO L_ST_IN_POS_PROFINET_OUT, 0;
                    ! Move Linear
                    SetGO L_ST_MAIN_ID_PROFINET_OUT, 40;
                ENDIF
                
                IF L_PLC_ST_ACTIVE_PROFINET_IN = 0 THEN
                    ! Initialization State {rm_str.PLC_ModuleOK = FALSE -> There is some problem with the PC's (PLC)}
                    SetGO L_ST_MAIN_ID_PROFINET_OUT, 10;
                ENDIF
                
            CASE 30: ! MOVE JOINT ABSOLUTE STATE - EXECUTE !
                ! Description: !
                !   Control Function (MoveAbsJ): Moves the robot to an absolute joint position !

                IF aux_trajectory_index_var = L_PLC_TRAJ_SIZE_PROFINET_IN THEN
                    SetDO L_ST_IN_POS_PROFINET_OUT, 1;
                    SetGO L_ST_MAIN_ID_PROFINET_OUT, 20;
                ELSE
                    SetGO L_MOTION_TRAJ_ID_PROFINET_OUT, aux_trajectory_index_var;
                    MoveAbsJ L_J_Position{aux_trajectory_index_var + 1}\NoEOffs, L_Speed{aux_trajectory_index_var + 1}, L_Zone{aux_trajectory_index_var + 1}, rob_L_Tool\WObj:=wobj0;
                    aux_trajectory_index_var := aux_trajectory_index_var + 1;
                ENDIF
                
            CASE 40: ! MOVE CARTESIAN STATE - EXECUTE !
                ! Description: !
                !   Control Function (MoveL): Moves the tool center point (TCP) linearly !

                IF aux_trajectory_index_var = L_PLC_TRAJ_SIZE_PROFINET_IN THEN
                    SetDO L_ST_IN_POS_PROFINET_OUT, 1;
                    SetGO L_ST_MAIN_ID_PROFINET_OUT, 20;
                ELSE
                    SetGO L_MOTION_TRAJ_ID_PROFINET_OUT, aux_trajectory_index_var;
                    MoveL L_TCP_Position{aux_trajectory_index_var + 1}, L_Speed{aux_trajectory_index_var + 1}, L_Zone{aux_trajectory_index_var + 1}, rob_L_Tool\WObj:=wobj0;
                    aux_trajectory_index_var := aux_trajectory_index_var + 1;
                ENDIF
        ENDTEST
    ENDPROC
    
    PROC ER_Reset_Parameters_MAIN_CTRL()
        ! Description:                                                   !
        !   Event routine function (non-parametric) for parameter reset. !
        !       Note: Configuration / Controller / Event Routine         !
        
        SetGO L_ST_MAIN_ID_PROFINET_OUT, 10;
        SetGO L_ST_UPT_ID_PROFINET_OUT, 10;
        
        ! Main Control Structure
        rm_L_str := [[0],[0,0,0,0],[[0,0],[0,0,0,0,0]],[0,0,0,0,0]];
                   
        ! Characteristics of a robotic tool (end-effector / gripper)
        rob_L_Tool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
	ENDPROC
    
    PROC ER_Reset_Parameters_Stopped()
        ! Description:                                                   !
        !   Event routine function (non-parametric) to reset parameters  !
        !   when the program is stopped.                                 !
        !       Note: Configuration / Controller / Event Routine         !
        
        SetDO L_ST_IN_POS_PROFINET_OUT, 1;
        SetGO L_ST_MAIN_ID_PROFINET_OUT, 10;
    ENDPROC
ENDMODULE