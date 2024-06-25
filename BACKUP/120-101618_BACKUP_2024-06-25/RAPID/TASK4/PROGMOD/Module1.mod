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
    ! File Name: T_ROB_PLC_PROFINET_TCP/Module1.mod
    ! ## =========================================================================== ## 

    ! Description: !
    !   TCP (Configuration - 1, 4, 6, X) !
    RECORD robot_data_tcp_cfg_sign_str
        num Cfg_1;
        num Cfg_4;
        num Cfg_6;
        num Cfg_X;
    ENDRECORD
    
    ! Description: !
    !   TCP (Position, Rotation - X, Y, Z) !
    RECORD robot_data_tcp_pr_sign_str
        num X;
        num Y;
        num Z;
    ENDRECORD
    
    ! Description: !
    !   TCP (Position, Rotation, Configuration): sign (+-) !
    RECORD robot_data_tcp_sign_str
        robot_data_tcp_pr_sign_str Position;
        robot_data_tcp_pr_sign_str Rotation;
        robot_data_tcp_cfg_sign_str Configuration;
        num External_Position;
    ENDRECORD
    
    ! Description: !
    !   The main structure of data collection on the current position of the robot with sign (+-) !
    RECORD robot_data_main_str
        robot_data_tcp_sign_str TCP_Sign; ! TCP (Position, Rotation - X, Y, Z): sign (+-)
        robtarget ROB_CURRENT_DATA;       ! Position Data (Cartesian)
    ENDRECORD
    
    ! Call Main Structure
    !   Note: VAR robot_data_main_str r_data := [[[0,0,0],[0,0,0],[0,0,0,0],0],[[0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0,0,0]]];
    VAR robot_data_main_str r_data;
    
    ! Characteristics of a robotic tool (end-effector / gripper)
    PERS tooldata rob_Tool;
    
    ! Variable for rounded (Joint, Cartesian) data
    VAR num NUM_OF_ROUNDED_PLACES := 2;
    
    ! Mathematical Constant PI
    VAR num M_PI := 3.14159;
    
    ! Accuracy factor for value conversion
    VAR num accuracy_factor := 100;
    VAR robtarget a;
    
    ! Description:                   !
    !   Program Main Cycle:          !
    !       Type        : Semistatic !
    !       TrustLeve   : No Safety  !
    !       Motion Task : N/A        !
    PROC Main()
        ! Description:                                                       !
        !   Read the the current Tool Center Point (TCP) of the robot.       ! 
        !       Note: The resulting data is rounded with the Round function  !
        
        r_data.ROB_CURRENT_DATA := CRobT(\Tool:=rob_Tool \WObj:=wobj0);
        
        ! TCP Position (X, Y, Z) -> X Parameter
        r_data.TCP_Sign.Position.X := sign(Round(r_data.ROB_CURRENT_DATA.trans.x \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDO TCP_POS_X_SIGN_PROFINET_OUT, r_data.TCP_Sign.Position.X;
        IF r_data.TCP_Sign.Position.X = 0 THEN
            SetGO TCP_POS_X_PROFINET_OUT, Round(((-1) * (r_data.ROB_CURRENT_DATA.trans.x * accuracy_factor)) \Dec:=0);
        ELSE  
            SetGO TCP_POS_X_PROFINET_OUT, Round((r_data.ROB_CURRENT_DATA.trans.x * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! TCP Position (X, Y, Z) -> Y Parameter
        r_data.TCP_Sign.Position.Y := sign(Round(r_data.ROB_CURRENT_DATA.trans.y \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDO TCP_POS_Y_SIGN_PROFINET_OUT, r_data.TCP_Sign.Position.Y;
        IF r_data.TCP_Sign.Position.Y = 0 THEN
            SetGO TCP_POS_Y_PROFINET_OUT, Round(((-1) * (r_data.ROB_CURRENT_DATA.trans.y * accuracy_factor)) \Dec:=0);
        ELSE  
            SetGO TCP_POS_Y_PROFINET_OUT, Round((r_data.ROB_CURRENT_DATA.trans.y * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! TCP Position (X, Y, Z) -> Z Parameter
        r_data.TCP_Sign.Position.Z := sign(Round(r_data.ROB_CURRENT_DATA.trans.z \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDO TCP_POS_Z_SIGN_PROFINET_OUT, r_data.TCP_Sign.Position.Z;
        IF r_data.TCP_Sign.Position.Z = 0 THEN
            SetGO TCP_POS_Z_PROFINET_OUT, Round(((-1) * (r_data.ROB_CURRENT_DATA.trans.z * accuracy_factor)) \Dec:=0);
        ELSE  
            SetGO TCP_POS_Z_PROFINET_OUT, Round((r_data.ROB_CURRENT_DATA.trans.z * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! Description:                                                                    !
        !   Calculation of EA (Euler Angles) from quaternions using the EulerZYX function !
        
        ! TCP Rotation (Euler Angles: X, Y, Z) -> X Parameter
        r_data.TCP_Sign.Rotation.X := sign(Round(EulerZYX(\X, r_data.ROB_CURRENT_DATA.rot) \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDO TCP_ROT_X_SIGN_PROFINET_OUT, r_data.TCP_Sign.Rotation.X;
        IF r_data.TCP_Sign.Rotation.X = 0 THEN
            SetGO TCP_ROT_X_PROFINET_OUT, Round(((-1) * (EulerZYX(\X, r_data.ROB_CURRENT_DATA.rot) * accuracy_factor)) \Dec:=0);
        ELSE  
            SetGO TCP_ROT_X_PROFINET_OUT, Round((EulerZYX(\X, r_data.ROB_CURRENT_DATA.rot) * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! TCP Rotation (Euler Angles: X, Y, Z) -> Y Parameter
        r_data.TCP_Sign.Rotation.Y := sign(Round(EulerZYX(\Y, r_data.ROB_CURRENT_DATA.rot) \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDO TCP_ROT_Y_SIGN_PROFINET_OUT, r_data.TCP_Sign.Rotation.Y;
        IF r_data.TCP_Sign.Rotation.Y = 0 THEN
            SetGO TCP_ROT_Y_PROFINET_OUT, Round(((-1) * (EulerZYX(\Y, r_data.ROB_CURRENT_DATA.rot) * accuracy_factor)) \Dec:=0);
        ELSE  
            SetGO TCP_ROT_Y_PROFINET_OUT, Round((EulerZYX(\Y, r_data.ROB_CURRENT_DATA.rot) * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! TCP Rotation (Euler Angles: X, Y, Z) -> Z Parameter
        r_data.TCP_Sign.Rotation.Z := sign(Round(EulerZYX(\Z, r_data.ROB_CURRENT_DATA.rot) \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDO TCP_ROT_Z_SIGN_PROFINET_OUT, r_data.TCP_Sign.Rotation.Z;
        IF r_data.TCP_Sign.Rotation.Z = 0 THEN
            SetGO TCP_ROT_Z_PROFINET_OUT, Round(((-1) * (EulerZYX(\Z, r_data.ROB_CURRENT_DATA.rot) * accuracy_factor)) \Dec:=0);
        ELSE  
            SetGO TCP_ROT_Z_PROFINET_OUT, Round((EulerZYX(\Z, r_data.ROB_CURRENT_DATA.rot) * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! Description:                                             !
        !   Expression of the configuration: Cfg -> 1, 4, 6, and X !
        
        ! TCP Configuration (1, 4, 6, X) -> Parameter 1
        r_data.TCP_Sign.Configuration.Cfg_1 := sign(r_data.ROB_CURRENT_DATA.robconf.cf1);
        SetDO TCP_CFG_1_SIGN_PROFINET_OUT, r_data.TCP_Sign.Configuration.Cfg_1;
        IF r_data.TCP_Sign.Configuration.Cfg_1 = 0 THEN
            SetGO TCP_CFG_1_PROFINET_OUT, (-1) * r_data.ROB_CURRENT_DATA.robconf.cf1;
        ELSE
            SetGO TCP_CFG_1_PROFINET_OUT, r_data.ROB_CURRENT_DATA.robconf.cf1;
        ENDIF
        
        ! TCP Configuration (1, 4, 6, X) -> Parameter 4
        r_data.TCP_Sign.Configuration.Cfg_4 := sign(r_data.ROB_CURRENT_DATA.robconf.cf4);
        SetDO TCP_CFG_4_SIGN_PROFINET_OUT, r_data.TCP_Sign.Configuration.Cfg_4;
        IF r_data.TCP_Sign.Configuration.Cfg_4 = 0 THEN
            SetGO TCP_CFG_4_PROFINET_OUT, (-1) * r_data.ROB_CURRENT_DATA.robconf.cf4;
        ELSE
            SetGO TCP_CFG_4_PROFINET_OUT, r_data.ROB_CURRENT_DATA.robconf.cf4;
        ENDIF
        
        ! TCP Configuration (1, 4, 6, X) -> Parameter 6
        r_data.TCP_Sign.Configuration.Cfg_6 := sign(r_data.ROB_CURRENT_DATA.robconf.cf6);
        SetDO TCP_CFG_6_SIGN_PROFINET_OUT, r_data.TCP_Sign.Configuration.Cfg_6;
        IF r_data.TCP_Sign.Configuration.Cfg_6 = 0 THEN
            SetGO TCP_CFG_6_PROFINET_OUT, (-1) * r_data.ROB_CURRENT_DATA.robconf.cf6;
        ELSE
            SetGO TCP_CFG_6_PROFINET_OUT, r_data.ROB_CURRENT_DATA.robconf.cf6;
        ENDIF
    
        ! TCP Configuration (1, 4, 6, X) -> Parameter X
        r_data.TCP_Sign.Configuration.Cfg_X := sign(r_data.ROB_CURRENT_DATA.robconf.cfx);
        SetDO TCP_CFG_X_SIGN_PROFINET_OUT, r_data.TCP_Sign.Configuration.Cfg_X;
        IF r_data.TCP_Sign.Configuration.Cfg_X = 0 THEN
            SetGO TCP_CFG_X_PROFINET_OUT, (-1) * r_data.ROB_CURRENT_DATA.robconf.cfx;
        ELSE
            SetGO TCP_CFG_X_PROFINET_OUT, r_data.ROB_CURRENT_DATA.robconf.cfx;
        ENDIF
        
        ! Description:                      !
        !   Expression of the external axis !
        
        ! TCP External Axis Position
        r_data.TCP_Sign.External_Position := sign(Round(r_data.ROB_CURRENT_DATA.extax.eax_a \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDO TCP_EX_POS_SIGN_PROFINET_OUT, r_data.TCP_Sign.External_Position;
        IF r_data.ROB_CURRENT_DATA.extax.eax_a <> 9E+09 THEN
            IF r_data.TCP_Sign.External_Position = 0 THEN
                SetGO TCP_EX_POS_PROFINET_OUT, Round(((-1) * (r_data.ROB_CURRENT_DATA.extax.eax_a * accuracy_factor)) \Dec:=0);
            ELSE  
                SetGO TCP_EX_POS_PROFINET_OUT, Round((r_data.ROB_CURRENT_DATA.extax.eax_a * accuracy_factor) \Dec:=0);
            ENDIF
        ENDIF
        
    ENDPROC
    
    FUNC num sign(num value)
        ! Description:                                                       !
        !   A simple mathematical function that extracts a real number sign. !
        !                                                                    !
        ! IN:                                                                !
        ! [1] value [num]: Real number.                                      !
        ! OUT:                                                               !
        ! [1] return [num]: Sign (1: Positive {+}, 0: Negative {-}).         !
        
        IF value >= 0 THEN
           RETURN 1;
        ELSE
           RETURN 0;
        ENDIF
    ENDFUNC
    
    PROC ER_Reset_Parameters_TCP()
        ! Description:                                                   !
        !   Event routine function (non-parametric) for parameter reset. !
        !       Note: Configuration / Controller / Event Routine         !
        
        ! Main Structure for data collection
        r_data := [[[0,0,0],[0,0,0],[0,0,0,0],0],[[0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0,0,0]]];
        
        ! Characteristics of a robotic tool (end-effector / gripper)
        rob_Tool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
	ENDPROC
ENDMODULE