MODULE Module1
    !SyncMoveOff syncT;
    !MoveAbsJ [[0,-130,30,0,40,0],[135,9E9,9E9,9E9,9E9,9E9]] \NoEOffs, v50, fine, tool0;
    CONST robtarget Target_Sync_Home:=[[310,25,175],[0.707106692,0.70710687,0.00000004,-0.000000131],[0,0,0,0],[-134.999973944,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_X_m30:=[[309.999996541,21.650322222,162.50018161],[0.866025341,0.500000109,0.000000073,-0.000000116],[0,0,0,0],[-134.999973944,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_X_p30:=[[310.000002599,21.650315927,187.499816703],[0.499999891,0.866025467,0.000000005,-0.000000137],[0,0,0,0],[-134.999973944,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_Y_m30:=[[322.499817119,21.650317479,174.999996138],[0.683012626,0.683012754,-0.18301264,-0.183012851],[0,0,0,0],[-134.999973944,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_Y_p30:=[[297.500182017,21.650320704,175.000002178],[0.683012606,0.683012822,0.183012717,0.183012598],[0,0,0,0],[-134.999973944,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
    ! PERS Variables -> Communication between multiple Tasks (T_ROB_L, T_ROB_R)
    PERS bool sync_anim_move;
    ! Initialization Tasks for synchronization
    PERS tasks Task_list{2} := [ ["T_ROB_L"], ["T_ROB_R"] ];
    
    VAR num state;
    VAR syncident syncT;
    
    ! Identifier for the EGM correction
    LOCAL VAR egmident egm_id;
    ! The work object. Base Frame
    LOCAL PERS wobjdata egm_wobj := [FALSE, TRUE, "", [[0.0, 0.0, 0.0],[1.0, 0.0, 0.0, 0.0]], [[0.0, 0.0, 0.0],[1.0, 0.0, 0.0, 0.0]]];
    ! Limits for convergence
    ! Orientation: +-0.1 degrees
    LOCAL CONST egm_minmax egm_condition_orient := [-0.1, 0.1];
    VAR num state_id := 0;
    
    PROC main()
        !SyncMoveOff syncT;
        MoveAbsJ [[0,-130,30,0,40,0],[135,9E9,9E9,9E9,9E9,9E9]] \NoEOffs, v50, fine, tool0;
        !Path_Test_L;
    ENDPROC
    
    PROC Path_Test_L()
        TEST state
            CASE 0:
                MoveJ Offs(Target_Sync_Home, 0,0,0), v100, fine, Servo;
                state := 1;
                
            CASE 1:
                state := 2;
            CASE 2:
                WaitSyncTask syncT, Task_list;
                SyncMoveOn syncT, Task_list;
                state := 3;
            CASE 3:
                ! 1\
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0, v20, fine, Servo;
                MoveL Offs(Target_Sync_Home, 0,50,0), \ID:=0, v20, fine, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v20, fine, Servo;
               
                ! 2\
                MoveL Offs(Target_Sync_X_p30, 0,0,0), \ID:=0,v20, fine, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v20, fine, Servo;
                MoveL Offs(Target_Sync_X_m30, 0,0,0), \ID:=0,v20, fine, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v20, fine, Servo;
                MoveL Offs(Target_Sync_Y_p30, 0,0,0), \ID:=0,v20, fine, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v20, fine, Servo;
                MoveL Offs(Target_Sync_Y_m30, 0,0,0), \ID:=0,v20, fine, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v20, fine, Servo;
                
                ! 3\a
                MoveL Offs(Target_Sync_Home, 0,50,0), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,50,50), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,50), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v50, fine, Servo;
                ! 3\b
                MoveL Offs(Target_Sync_Home, 0,0,50), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,50,50), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,50,0), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v100, fine, Servo;
                
                ! 4\a
                MoveL Offs(Target_Sync_Home, 50,0,0), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 50,0,50), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,50), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v50, fine, Servo;
                ! 4\b
                MoveL Offs(Target_Sync_Home, 0,0,50), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 50,0,50), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 50,0,0), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v100, fine, Servo;

                ! 5\a
                MoveL Offs(Target_Sync_Home, 0,50,0), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,50,50), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,-50,50), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,-50,0), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0, v50, fine, Servo;
                ! 5\b
                MoveL Offs(Target_Sync_Home, 0,-50,0), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,-50,-50), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,50,-50), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,50,0), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0, v100, z20, Servo;
                
                
                state := 4;
            CASE 4:
                state := 5;
            CASE 5:
                SyncMoveOff syncT;
                state := 6;
            CASE 6:
                state := 2;
        ENDTEST
    ENDPROC
    
    PROC Path_20()
        MoveL Target_Sync_Home,v1000,z100,Servo\WObj:=wobj0;
        MoveL Target_Sync_X_m30,v1000,z100,Servo\WObj:=wobj0;
        MoveL Target_Sync_X_p30,v1000,z100,Servo\WObj:=wobj0;
        MoveL Target_Sync_Y_m30,v1000,z100,Servo\WObj:=wobj0;
        MoveL Target_Sync_Y_p30,v1000,z100,Servo\WObj:=wobj0;
    ENDPROC
ENDMODULE