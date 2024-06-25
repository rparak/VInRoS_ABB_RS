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
    ! File Name: T_PLC_ROB_PROFINET/Module1.mod
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
    PERS robot_ctrl_str rm_str;
    
    ! Joint Position Data
    PERS jointtarget J_Position{100};
    
    ! TCP Position Data
    PERS robtarget TCP_Position{100};

    ! Joint Speed Data
    PERS speeddata Speed{100};
    
    ! Joint Zone Data
    PERS zonedata Zone{100};

    ! Characteristics of a robotic tool (end-effector / gripper)
    PERS tooldata rob_Tool;

    VAR num actual_traj_id;
    VAR num aux_traj_id;
    
    ! Mathematical Constant PI
    VAR num M_PI := 3.14159265358979323846264338327950288;
 
    VAR num update_state;
    
    ! Description:                   !
    !   Program Main Cycle:          !
    !       Type        : Semistatic !
    !       TrustLeve   : No Safety  !
    !       Motion Task : N/A        !
    PROC Main()
        TEST ROB_ST_UPT_ID_PROFINET_OUT
            CASE 10: ! WAIT STATE !
                ! Description: !
                !   State of waiting for command with index from the PC's (PLC) !
                
                ! Reset parameters
                actual_traj_id := 0;
                aux_traj_id    := 0;
                
                ! Initialize the internal value of the accuracy conversion factor
                rm_str.internal.accuracy_factor            := 100;
                rm_str.internal.accuracy_factor_QUATERNION := 100000000;
                
                ! Information that the robot side is already active
                SetDO ROB_ST_ACTIVE_PROFINET_OUT, 1;
                
                IF PLC_CMD_UTP_PROFINET_IN = 1 AND PLC_CMD_UPT_ID_PROFINET_IN = 1 THEN
                    ! Update Trajectory Parameters (Joint)
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 40;
                ELSEIF PLC_CMD_UTP_PROFINET_IN = 1 AND PLC_CMD_UPT_ID_PROFINET_IN = 2 THEN
                    ! Update Trajectory Parameters (TCP)
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 50;
                ELSEIF PLC_CMD_UTP_PROFINET_IN = 1 AND PLC_CMD_UPT_ID_PROFINET_IN = 10 THEN
                    ! Update Tool Parameters
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 100;
                ENDIF
                
            CASE 40: ! UPDATE JOINT PARAMETERS STATE - CHECK !
                IF PLC_TRAJ_SIZE_PROFINET_IN = actual_traj_id THEN
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 10;
                ELSE
                    aux_traj_id := actual_traj_id;
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 41;
                ENDIF
               
            CASE 41: ! UPDATE JOINT PARAMETERS STATE - INCREASE INDEX !
                IF actual_traj_id = aux_traj_id THEN
                    actual_traj_id := aux_traj_id + 1;
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 42;
                ENDIF
                
            CASE 42: ! UPDATE JOINT PARAMETERS STATE - INFORM PLC !
                IF PLC_ST_UPDATE_DONE_PROFINET_IN = 1 THEN
                    SetDO ROB_ST_UPDATE_DONE_PROFINET_OUT, 0;
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 43;
                ENDIF
 
            CASE 43: ! UPDATE JOINT PARAMETERS STATE - EXECUTE !
                ! Set parameters for the specified motion type !
                ! SPEED / ZONE
                Speed{actual_traj_id} := set_speeddata(SPEED_DATA_PROFINET_IN); Zone{actual_traj_id} := set_zonedata(ZONE_DATA_PROFINET_IN);
                ! JOINT POSITION
                ! Q0 ... Q5
                J_Position{actual_traj_id}.robax.rax_1 := sign(Q0_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(Q0_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                J_Position{actual_traj_id}.robax.rax_2 := sign(Q1_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(Q1_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                J_Position{actual_traj_id}.robax.rax_3 := sign(Q2_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(Q2_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                J_Position{actual_traj_id}.robax.rax_4 := sign(Q3_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(Q3_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                J_Position{actual_traj_id}.robax.rax_5 := sign(Q4_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(Q4_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                J_Position{actual_traj_id}.robax.rax_6 := sign(Q5_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(Q5_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                ! Q0 - External
                J_Position{actual_traj_id}.extax.eax_a := sign(Q6_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(Q6_PROFINET_IN))) / rm_str.internal.accuracy_factor);

                IF PLC_ST_UPDATE_DONE_PROFINET_IN = 0 THEN
                    SetDO ROB_ST_UPDATE_DONE_PROFINET_OUT, 1;
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 40;
                ENDIF
                
            CASE 50: ! UPDATE TCP PARAMETERS STATE - INCREASE INDEX !
                IF PLC_TRAJ_SIZE_PROFINET_IN = actual_traj_id THEN
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 10;
                ELSE
                    aux_traj_id := actual_traj_id;
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 51;
                ENDIF
               
            CASE 51: ! UPDATE TCP PARAMETERS STATE - CHECK !
                IF actual_traj_id = aux_traj_id THEN
                    actual_traj_id := aux_traj_id + 1;
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 52;
                ENDIF
                
            CASE 52: ! UPDATE TCP PARAMETERS STATE - INFORM PLC !
                IF PLC_ST_UPDATE_DONE_PROFINET_IN = 1 THEN
                    SetDO ROB_ST_UPDATE_DONE_PROFINET_OUT, 0;
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 53;
                ENDIF
                
            CASE 53: ! UPDATE TCP PARAMETERS STATE - EXECUTE !
                ! Set parameters for the specified motion type !
                ! SPEED / ZONE
                Speed{actual_traj_id} := set_speeddata(SPEED_DATA_PROFINET_IN); Zone{actual_traj_id} := set_zonedata(ZONE_DATA_PROFINET_IN);
                ! TCP (Tool Center Point)
                ! Position
                TCP_Position{actual_traj_id}.trans.x := sign(TCP_POS_X_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(TCP_POS_X_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                TCP_Position{actual_traj_id}.trans.y := sign(TCP_POS_Y_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(TCP_POS_Y_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                TCP_Position{actual_traj_id}.trans.z := sign(TCP_POS_Z_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(TCP_POS_Z_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                ! Rotation
                TCP_Position{actual_traj_id}.rot := OrientZYX(sign(TCP_ROT_X_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(TCP_ROT_X_PROFINET_IN))) / rm_str.internal.accuracy_factor),
                                                              sign(TCP_ROT_Y_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(TCP_ROT_Y_PROFINET_IN))) / rm_str.internal.accuracy_factor),
                                                              sign(TCP_ROT_Z_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(TCP_ROT_Z_PROFINET_IN))) / rm_str.internal.accuracy_factor));
                ! Configuration
                TCP_Position{actual_traj_id}.robconf.cf1 := sign(TCP_CFG_1_SIGN_PROFINET_IN) * (TCP_CFG_1_PROFINET_IN); 
                TCP_Position{actual_traj_id}.robconf.cf4 := sign(TCP_CFG_4_SIGN_PROFINET_IN) * (TCP_CFG_4_PROFINET_IN);
                TCP_Position{actual_traj_id}.robconf.cf6 := sign(TCP_CFG_6_SIGN_PROFINET_IN) * (TCP_CFG_6_PROFINET_IN);
                TCP_Position{actual_traj_id}.robconf.cfx := sign(TCP_CFG_X_SIGN_PROFINET_IN) * (TCP_CFG_X_PROFINET_IN);
                ! External Position
                TCP_Position{actual_traj_id}.extax.eax_a := sign(TCP_EX_POS_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(TCP_EX_POS_PROFINET_IN))) / rm_str.internal.accuracy_factor);
                
                IF PLC_ST_UPDATE_DONE_PROFINET_IN = 0 THEN
                    SetDO ROB_ST_UPDATE_DONE_PROFINET_OUT, 1;
                    SetGO ROB_ST_UPT_ID_PROFINET_OUT, 50;
                ENDIF
                
            CASE 100: ! UPDATE TOOL PARAMETERS STATE !
                ! Description: !
                !   Define the parameters of the robotic tool !
                
                ! The robot is holding the tool (TRUE / FALSE)
                IF RTOOL_RH_PROFINET_IN = 1 THEN
                    rob_Tool.robhold := TRUE;
                
                    ! Translation position
                    rob_Tool.tframe.trans := [sign(RTOOL_X_POS_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_X_POS_PROFINET_IN))) / rm_str.internal.accuracy_factor), 
                                             sign(RTOOL_Y_POS_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_Y_POS_PROFINET_IN))) / rm_str.internal.accuracy_factor), 
                                             sign(RTOOL_Z_POS_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_Z_POS_PROFINET_IN))) / rm_str.internal.accuracy_factor)];
                    ! Rotation Position
                    rob_Tool.tframe.rot := [sign(RTOOL_QX_ROT_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_QX_ROT_PROFINET_IN))) / rm_str.internal.accuracy_factor_QUATERNION), 
                                           sign(RTOOL_QY_ROT_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_QY_ROT_PROFINET_IN))) / rm_str.internal.accuracy_factor_QUATERNION),
                                           sign(RTOOL_QZ_ROT_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_QZ_ROT_PROFINET_IN))) / rm_str.internal.accuracy_factor_QUATERNION),      
                                           sign(RTOOL_QW_ROT_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_QW_ROT_PROFINET_IN))) / rm_str.internal.accuracy_factor_QUATERNION)];
                    ! Tool Mass
                    rob_Tool.tload.mass := GInput(RTOOL_MASS_PROFINET_IN) / rm_str.internal.accuracy_factor;
                    
                    ! Center of Gravity
                    rob_Tool.tload.cog := [sign(RTOOL_X_COG_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_X_COG_PROFINET_IN))) / rm_str.internal.accuracy_factor), 
                                          sign(RTOOL_Y_COG_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_Y_COG_PROFINET_IN))) / rm_str.internal.accuracy_factor), 
                                          sign(RTOOL_Z_COG_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_Z_COG_PROFINET_IN))) / rm_str.internal.accuracy_factor)];
                    ! Inertial axes of tool load
                    rob_Tool.tload.aom := [sign(RTOOL_QX_AOM_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_QX_AOM_PROFINET_IN))) / rm_str.internal.accuracy_factor_QUATERNION), 
                                          sign(RTOOL_QY_AOM_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_QY_AOM_PROFINET_IN))) / rm_str.internal.accuracy_factor_QUATERNION), 
                                          sign(RTOOL_QZ_AOM_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_QZ_AOM_PROFINET_IN))) / rm_str.internal.accuracy_factor_QUATERNION), 
                                          sign(RTOOL_QW_AOM_SIGN_PROFINET_IN) * ((DnumToNum(GInputDnum(RTOOL_QW_AOM_PROFINET_IN))) / rm_str.internal.accuracy_factor_QUATERNION)];
                    ! Momets of inertia
                    rob_Tool.tload.ix := DnumToNum(GInputDnum(RTOOL_IX_MOI_PROFINET_IN)) / rm_str.internal.accuracy_factor;
                    rob_Tool.tload.iy := DnumToNum(GInputDnum(RTOOL_IY_MOI_PROFINET_IN)) / rm_str.internal.accuracy_factor;
                    rob_Tool.tload.iz := DnumToNum(GInputDnum(RTOOL_IZ_MOI_PROFINET_IN)) / rm_str.internal.accuracy_factor;
                ELSE
                    rob_Tool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
                ENDIF
                
                ! The parameter update is completed successfully
                PulseDO ROB_ST_UPDATE_DONE_PROFINET_OUT;
                
                SetGO ROB_ST_UPT_ID_PROFINET_OUT, 10;
        ENDTEST
    ENDPROC
    
    FUNC num sign(num value)
        ! Description:                                                                !
        !   A simple mathematical function that extracts a sign from a digital value. !
        !                                                                             !
        ! IN:                                                                         !
        ! [1] value [num]: Digital Input (1 / 0).                                     !
        ! OUT:                                                                        !
        ! [1] return [num]: Sign (1: Positive {+}, -1: Negative {-}).                 !
        
        IF value = 1 THEN
           RETURN 1;
        ELSE
           RETURN -1;
        ENDIF
    ENDFUNC
    
    FUNC num RadToDeg(num value)
        ! Description:                                       !
        !   Conversion function between radians and degrees. !
        !                                                    !
        ! IN:                                                !
        ! [1] value [num]: Real number (Radian).             !
        ! OUT:                                               !
        ! [1] return [num]: Degree.                          !
        
        RETURN ((value) * 180.0 / M_PI);
    ENDFUNC
    
    FUNC speeddata set_speeddata(num value)
        ! Description:                                                                                               !
        !   The function to specify the velocity at which both the robot and the external axes move.                 !
        !                                                                                                            !
        ! IN:                                                                                                        !
        ! [1] value [num]: Specified value from the PC's (PLC).                                                      !
        ! OUT:                                                                                                       !
        ! [1] return [speeddata {Velocity: TCP, Orientation, Linear External Axes, Rotational External Axes}]: 
        !                              Predefined speed data to be used for moving the robot and the external axes . !
        
        TEST value
            CASE 0:
                RETURN [5, 50, 1000, 500];
            CASE 1:
                RETURN [10, 50, 1000, 500];
            CASE 2:
                RETURN [20, 50, 1000, 500];
            CASE 3:
                RETURN [30, 50, 1000, 500];
            CASE 4:
                RETURN [40, 100, 1000, 500];
            CASE 5:
                RETURN [50, 100, 1000, 500];
            CASE 6:
                RETURN [60, 100, 1000, 500];
            CASE 7:
                RETURN [80, 100, 1000, 500];
            CASE 8:
                RETURN [100, 200, 1000, 500];
            CASE 9:
                RETURN [150, 200, 1000, 500];
            CASE 10:
                RETURN [200, 200, 1000, 500];
            CASE 11:
                RETURN [300, 200, 1000, 500];
            CASE 12:
                RETURN [400, 300, 1000, 500];
            CASE 13:
                RETURN [500, 300, 1000, 500];
            CASE 14:
                RETURN [600, 300, 1000, 500];
            CASE 15:
                RETURN [800, 300, 1000, 500];
            CASE 16:
                RETURN [1000, 400, 1000, 500];
            CASE 17:
                RETURN [1500, 400, 1000, 500];
            CASE 18:
                RETURN [2000, 400, 1000, 500];
            CASE 19:
                RETURN [2500, 400, 1000, 500];
            CASE 20:
                RETURN [3000, 500, 1000, 500];
            CASE 21:
                RETURN [4000, 500, 1000, 500];
            CASE 22:
                RETURN [5000, 500, 1000, 500];
            CASE 23:
                RETURN [6000, 500, 1000, 500];         
        ENDTEST
    ENDFUNC
    
    FUNC zonedata set_zonedata(num value)
        ! Description:                                                                                            !
        !   The function to specify how a position is to be terminated, i.e. how close to the programmed position 
        !   the axes must be before moving towards the next position.                                             !
        !                                                                                                         !
        ! IN:                                                                                                     !
        ! [1] value [num]: Specified value from the PC's (PLC).                                                   !
        ! OUT:                                                                                                    !
        ! [1] return [zonedata]: Defines whether the movement is to terminate as a stop point (fine point) or 
        !                        as a fly-by point.                                                               !
        !                           Stop points : Use zonedata named fine.                                        !
        !                           Other       : Fly-by points(z0 {0.3 mm / 0.03°} .. z200 {200 mm / 30°})       !

        TEST value
            CASE 0:
                RETURN fine;
            CASE 1:
                RETURN z0;
            CASE 2:
                RETURN z1;
            CASE 3:
                RETURN z5;
            CASE 4:
                RETURN z10;
            CASE 5:
                RETURN z15;
            CASE 6:
                RETURN z20;
            CASE 7:
                RETURN z30;
            CASE 8:
                RETURN z40;
            CASE 9:
                RETURN z50;
            CASE 10:
                RETURN z60;
            CASE 11:
                RETURN z80;
            CASE 12:
                RETURN z100;
            CASE 13:
                RETURN z150;
            CASE 14:
                RETURN z200;       
        ENDTEST
    ENDFUNC
ENDMODULE