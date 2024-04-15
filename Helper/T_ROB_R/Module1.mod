MODULE Module1
    CONST robtarget Target_Sync_Home:=[[310,-25,175],[0.707106696,-0.707106867,0.000000054,0.000000136],[0,0,0,0],[107.879991245,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_X_m30:=[[310.000002965,-21.650316051,187.499816736],[0.499999895,-0.866025464,0.000000017,0.000000145],[0,0,0,0],[107.879991245,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_X_p30:=[[309.999996267,-21.65032211,162.50018164],[0.866025343,-0.500000105,0.000000087,0.000000117],[0,0,0,0],[107.879991245,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_Y_m30:=[[297.500182066,-21.65032052,175.000002544],[0.683012605,-0.68301282,0.183012732,-0.183012593],[0,0,0,0],[107.879991245,9E+09,9E+09,9E+09,9E+09,9E+09]];
    CONST robtarget Target_Sync_Y_p30:=[[322.499817158,-21.650317625,174.999995846],[0.683012633,-0.683012749,-0.183012628,0.183012855],[0,0,0,0],[107.879991245,9E+09,9E+09,9E+09,9E+09,9E+09]];
    
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
        MoveAbsJ [[0,-130,30,0,40,0], [-135,9E9,9E9,9E9,9E9,9E9]]\NoEOffs, v50, fine, tool0;    
        !Path_Test_R;
    ENDPROC
    
    PROC Path_Test_R()
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
                MoveL Offs(Target_Sync_Home, 0,-50,0), \ID:=0, v20, fine, Servo;
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
                MoveL Offs(Target_Sync_Home, 0,-50,0), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,-50,50), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,50), \ID:=0, v50, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,0,0), \ID:=0,v50, fine, Servo;
                ! 3\b
                MoveL Offs(Target_Sync_Home, 0,0,50), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,-50,50), \ID:=0, v100, z20, Servo;
                MoveL Offs(Target_Sync_Home, 0,-50,0), \ID:=0, v100, z20, Servo;
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
    
    PROC Path_10()
        MoveL Target_Sync_Home,v1000,z100,Servo\WObj:=wobj0;
        MoveL Target_Sync_X_m30,v1000,z100,Servo\WObj:=wobj0;
        MoveL Target_Sync_X_p30,v1000,z100,Servo\WObj:=wobj0;
        MoveL Target_Sync_Y_p30,v1000,z100,Servo\WObj:=wobj0;
        MoveL Target_Sync_Y_m30,v1000,z100,Servo\WObj:=wobj0;
    ENDPROC
ENDMODULE