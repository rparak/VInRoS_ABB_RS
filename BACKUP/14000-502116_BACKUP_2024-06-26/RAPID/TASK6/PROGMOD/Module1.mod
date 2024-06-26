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
    ! File Name: T_ROB_PLC_PROFINET_Q/Module1.mod
    ! ## =========================================================================== ## 
    
    ! Description: !
    !   Joint Position Data (Q0 - Q5): sign (+-) !
    RECORD robot_data_joint_sign_str
        num Q0;
        num Q1;
        num Q2;
        num Q3;
        num Q4;
        num Q5;
        num Q6;
    ENDRECORD
    
    ! Description: !
    !   The main structure of data collection on the current position of the robot with sign (+-) !
    RECORD robot_data_main_str
        robot_data_joint_sign_str Joint_Sign; ! Joint Position Data (Q0 - Q6): sign (+-)
        jointtarget JOINT_CURRENT_DATA;       ! Joint Position Data
    ENDRECORD
    
    ! Call Main Structure
    !   Note: VAR robot_data_main_str rob_R_data := [[0,0,0,0,0,0,0],[[0,0,0,0,0,0],[0,0,0,0,0,0]]];
    VAR robot_data_main_str rob_R_data;
    
    ! Characteristics of a robotic tool (end-effector / gripper)
    !PERS tooldata rob_R_Tool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
    PERS tooldata rob_R_Tool;
    
    ! Variable for rounded (Joint, Cartesian) data
    VAR num NUM_OF_ROUNDED_PLACES := 2;
    
    ! Mathematical Constant PI
    VAR num M_PI := 3.14159;
    
    ! Accuracy factor for value conversion
    VAR num accuracy_factor := 100;

    ! Description:                   !
    !   Program Main Cycle:          !
    !       Type        : Semistatic !
    !       TrustLeve   : No Safety  !
    !       Motion Task : N/A        !
    PROC Main()
        ! Description:                                                       !
        !   Read the current angles of the robot axes (Joint).               ! 
        !       Note: The resulting data is rounded with the Round function  !
        
        rob_R_data.JOINT_CURRENT_DATA := CJointT();
        
        ! Joint (Q0 .. Q5) -> Q0 Parameter (Joint Number 0)
        rob_R_data.Joint_Sign.Q0 := sign(Round(rob_R_data.JOINT_CURRENT_DATA.robax.rax_1 \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDo R_Q0_SIGN_PROFINET_OUT, rob_R_data.Joint_Sign.Q0;
        IF rob_R_data.Joint_Sign.Q0 = 0 THEN
            SetGO R_Q0_PROFINET_OUT, Round(((-1) * (rob_R_data.JOINT_CURRENT_DATA.robax.rax_1 * accuracy_factor)) \Dec:=0);
        ELSE
            SetGO R_Q0_PROFINET_OUT, Round((rob_R_data.JOINT_CURRENT_DATA.robax.rax_1 * accuracy_factor) \Dec:=0);
        ENDIF
        !   Q0 Parameter (External)
        rob_R_data.Joint_Sign.Q6 := sign(Round(rob_R_data.JOINT_CURRENT_DATA.extax.eax_a \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDo R_Q6_SIGN_PROFINET_OUT, rob_R_data.Joint_Sign.Q6;
        IF rob_R_data.JOINT_CURRENT_DATA.extax.eax_a <> 9E+09 THEN
            IF rob_R_data.Joint_Sign.Q6 = 0 THEN
                SetGO R_Q6_PROFINET_OUT, Round(((-1) * (rob_R_data.JOINT_CURRENT_DATA.extax.eax_a * accuracy_factor)) \Dec:=0);
            ELSE
                SetGO R_Q6_PROFINET_OUT, Round((rob_R_data.JOINT_CURRENT_DATA.extax.eax_a * accuracy_factor) \Dec:=0);
            ENDIF
        ENDIF
        
        ! Joint (Q0 .. Q5) -> Q1 Parameter (Joint Number 1)
        rob_R_data.Joint_Sign.Q1 := sign(Round(rob_R_data.JOINT_CURRENT_DATA.robax.rax_2 \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDo R_Q1_SIGN_PROFINET_OUT, rob_R_data.Joint_Sign.Q1;
        IF rob_R_data.Joint_Sign.Q1 = 0 THEN
            SetGO R_Q1_PROFINET_OUT, Round(((-1) * (rob_R_data.JOINT_CURRENT_DATA.robax.rax_2 * accuracy_factor)) \Dec:=0);
        ELSE
            SetGO R_Q1_PROFINET_OUT, Round((rob_R_data.JOINT_CURRENT_DATA.robax.rax_2 * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! Joint (Q0 .. Q5) -> Q2 Parameter (Joint Number 2)
        rob_R_data.Joint_Sign.Q2 := sign(Round(rob_R_data.JOINT_CURRENT_DATA.robax.rax_3 \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDo R_Q2_SIGN_PROFINET_OUT, rob_R_data.Joint_Sign.Q2;
        IF rob_R_data.Joint_Sign.Q2 = 0 THEN
            SetGO R_Q2_PROFINET_OUT, Round(((-1) * (rob_R_data.JOINT_CURRENT_DATA.robax.rax_3 * accuracy_factor)) \Dec:=0);
        ELSE
            SetGO R_Q2_PROFINET_OUT, Round((rob_R_data.JOINT_CURRENT_DATA.robax.rax_3 * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! Joint (Q0 .. Q5) -> Q3 Parameter (Joint Number 3)
        rob_R_data.Joint_Sign.Q3 := sign(Round(rob_R_data.JOINT_CURRENT_DATA.robax.rax_4 \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDo R_Q3_SIGN_PROFINET_OUT, rob_R_data.Joint_Sign.Q3;
        IF rob_R_data.Joint_Sign.Q3 = 0 THEN
            SetGO R_Q3_PROFINET_OUT, Round(((-1) * (rob_R_data.JOINT_CURRENT_DATA.robax.rax_4 * accuracy_factor)) \Dec:=0);
        ELSE
            SetGO R_Q3_PROFINET_OUT, Round((rob_R_data.JOINT_CURRENT_DATA.robax.rax_4 * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! Joint (Q0 .. Q5) -> Q4 Parameter (Joint Number 4)
        rob_R_data.Joint_Sign.Q4 := sign(Round(rob_R_data.JOINT_CURRENT_DATA.robax.rax_5 \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDo R_Q4_SIGN_PROFINET_OUT, rob_R_data.Joint_Sign.Q4;
        IF rob_R_data.Joint_Sign.Q4 = 0 THEN
            SetGO R_Q4_PROFINET_OUT, Round(((-1) * (rob_R_data.JOINT_CURRENT_DATA.robax.rax_5 * accuracy_factor)) \Dec:=0);
        ELSE
            SetGO R_Q4_PROFINET_OUT, Round((rob_R_data.JOINT_CURRENT_DATA.robax.rax_5 * accuracy_factor) \Dec:=0);
        ENDIF
        
        ! Joint (Q0 .. Q5) -> Q5 Parameter (Joint Number 5)
        rob_R_data.Joint_Sign.Q5 := sign(Round(rob_R_data.JOINT_CURRENT_DATA.robax.rax_6 \Dec:=NUM_OF_ROUNDED_PLACES));
        SetDo R_Q5_SIGN_PROFINET_OUT, rob_R_data.Joint_Sign.Q5;
        IF rob_R_data.Joint_Sign.Q5 = 0 THEN
            SetGO R_Q5_PROFINET_OUT, Round(((-1) * (rob_R_data.JOINT_CURRENT_DATA.robax.rax_6 * accuracy_factor)) \Dec:=0);
        ELSE
            SetGO R_Q5_PROFINET_OUT, Round((rob_R_data.JOINT_CURRENT_DATA.robax.rax_6 * accuracy_factor) \Dec:=0);
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
    
    PROC ER_Reset_Parameters_Q()
        ! Description:                                                   !
        !   Event routine function (non-parametric) for parameter reset. !
        !       Note: Configuration / Controller / Event Routine         !
        
        ! Main Structure for data collection
        rob_R_data := [[0,0,0,0,0,0,0],[[0,0,0,0,0,0],[0,0,0,0,0,0]]];
        
        ! Characteristics of a robotic tool (end-effector / gripper)
        rob_R_Tool := [TRUE,[[0,0,0],[1,0,0,0]],[0.001,[0,0,0.001],[1,0,0,0],0,0,0]];
	ENDPROC
ENDMODULE