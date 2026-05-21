.PROGRAM main1()
; ============================
; ロボット① メインプログラム
; ============================

; ---- 起動処理 ----
WAIT PLC_START = ON         ; PLC起動信号待ち
JMOVE HOME                   ; 原点確認

; ---- 計測トリガー ----
SIGNAL VISION_START         ; ビジョン計測開始
SIGNAL ROB2_MEASURE_REQ     ; ロボ②へ計測指令

; ---- ATC前ポジション移動 ----
JMOVE ATC_PRE_POS            ; ハンド装着直前位置へ移動

; ---- ビジョン結果待ち ----
WAIT VISION_DONE = ON       ; ビジョン完了待ち

; ---- ビジョン結果取得 ----

; ---- ビジョン判定 ----
IF VISION_OK == OFF THEN    ; ビジョン計測異常
    SIGNAL PLC_ERROR        ; PLCへ異常通知
    JMOVE HOME               ; 原点復帰
    STOP                    ; エラー処理
ENDIF

; ---- ハンド装着 ----
JMOVE ATC_ATTACH_POS
TOOL_ATTACH                 ; ハンド装着

; ---- ワーク①処理 ----
JMOVE WORK1_PICK_POS    ;把持位置へ移動
GRIP                        ; ワーク①把持

JMOVE WORK1_PLACE_POS   ;配置位置へ移動
RELEASE                     ; ワーク①配置

; ---- 補正値受信 ----
SIGNAL OFFSET_REQ           ; PLCへ補正値要求
WAIT OFFSET_READY = ON      ; 補正値受信待ち

; ---- 補正値判定 ----
IF OFFSET_OK == OFF THEN
    SIGNAL PLC_ERROR        ; PLCへ異常通知
    ; （エラー処理：TOPのみ組立など）
    STOP
ENDIF

; ---- ワーク②処理 ----
JMOVE WORK2_PICK_POS
GRIP

; 補正適用移動
JMOVE WORK2_PLACE_POS + OFFSET(X,Y,θ)
RELEASE

; ---- 配置判定 ----
IF PLACE_OK == OFF THEN
    SIGNAL PLC_ERROR
    STOP
ENDIF

; ---- 完了通知 ----
SIGNAL PLC_COMPLETE         ; PLCへ設置完了

; ---- 原点復帰 ----
JMOVE HOME

; ---- サイクル継続判定 ----
WAIT PLC_NEXT_CMD           ; PLC判断待ち

IF PLC_CONTINUE == ON THEN
    ; ハンド保持
    JMP START               ; 次サイクル
ELSE
    ; ハンド返却
    JMOVE ATC_DETACH_POS
    TOOL_DETACH
    JMOVE HOME
    END
ENDIF
.END




;**********************************************************************************************************************
;ロボット動作サブルーチン
;**********************************************************************************************************************

;***********************************************************
;初期化処理関連
;***********************************************************

.PROGRAM z.init() ; 初期設定
; /======================================================================/
; FUNCTION: 初期設定
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  CHECK.HOLD OFF
  CP ON
  CYCLE.STOP OFF
  OX.PREOUT ON
  PREFETCH.SIGINS OFF
  QTOOL OFF
  REP_ONCE ON
  RPS ON
  STP_ONCE OFF
  MESSAGES ON
  SCREEN ON
  AUTOSTART.PC ON
  AUTOSTART2.PC OFF
  AUTOSTART3.PC OFF
  AUTOSTART4.PC OFF
  AUTOSTART5.PC OFF
  ABS.SPEED OFF

;TOOL NULL

  RESET ; 出力ﾘｾｯﾄ(内部信号はﾘｾｯﾄされない)
;
;偏差異常ﾁｪｯｸON
  CGNENVCHKON
;重力補償値(kg,重心toolX,重心toolY,重心toolZ)
  CGNGRAV 71,0,-300,0 ; 仮
;
  CALL z.init_real
  CALL z.init_io
;
;侵入領域の制限
  il_skipflg = OFF ; ｲﾝﾀｰﾛｯｸ確認ｽｷｯﾌﾟﾌﾗｸﾞ　REAC,TOPのRB単体締結動作時にON
;
;<< PCﾌﾟﾛｸﾞﾗﾑ起動ﾁｪｯｸ >>
  IF TASK(1001)<>1 THEN ; PC1起動中以外
    PCEXECUTE autostart.pc
    TWAIT 1
    IF TASK(1001)<>1 THEN ; PC1起動中以外
      CALL z.app_error("autostart.pc停止ｴﾗｰ")
      HALT
    END
  END
;
  RETURN
;
.END
.PROGRAM z.init_io() #40; I/O設定
; /======================================================================/
; FUNCTION: I/O設定
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
;<< 出力信号 >>
  ox_motor_on = 1 ; ﾓｰﾀ電源ON中
  ox_error = 2 ; ｴﾗｰ発生中
  ox_stable = 3 ; 定常状態
  ox_cycle = 4 ; 自動運転中
  ox_teachmode = 5 ; ﾃｨｰﾁﾓｰﾄﾞ
  ox_home1 = 6 ; 第一原点
  ox_home2 = 7 ; 第二原点
  ox_rps_on = 8 ; 外部プログラム選択(RPS)有効
  ox_teachlock = 9 ; ﾃｨｰﾁﾛｯｸ入
  ox_emg_on = 10 ; 非常停止中
  ox_hold = 11 ; ﾎｰﾙﾄﾞ中
  ox_app_error = 12 ; ｱﾌﾟﾘｴﾗｰ発生中
;
  FOR .i = 0 TO 13 ; ﾛﾎﾞｯﾄPG番号ｱﾝｻｰ14bit
    ox_pgno_ans[.i+1] = 17+.i
  END
;
  ox_wait_restart = 33 ; 評価再開待ち中
  ox_wait_nmashi = 34 ; 評価N増し待ち中
  ox_wait_finish = 35 ; 評価終了待ち中
  ox_complete = 36 ; 評価作業完了
;
  ox_carry = 38 ; ﾕﾆｯﾄ搬送中(ﾏﾃﾊﾝｾﾝｻPLC常時監視用)
;
  FOR .i = 0 TO 6 ; ﾁｪﾝｼﾞｹﾞｲﾝ位置指定7bit
    ox_gain_pos[.i+1] = 41+.i
  END
;
  ox_trg_vis = 49 ; ﾋﾞｼﾞｮﾝ座標PLC出力ﾄﾘｶﾞ
  FOR .i = 0 TO 6 ; ﾋﾞｼﾞｮﾝﾛｸﾞ位置選択7bit
    ox_vis_log[.i+1] = 50+.i
  END
;
  FOR .i = 0 TO 6 ; ﾋﾞｼﾞｮﾝ補正ID取得7bit
    ox_vis_pos[.i+1] = 57+.i
  END
;
  ox_object_ans[1] = 65 ; 評価対象番号ｱﾝｻｰ:RF UNIT
  ox_object_ans[2] = 66 ; 評価対象番号ｱﾝｻｰ:EXHAUST UNIT
  ox_object_ans[3] = 67 ; 評価対象番号ｱﾝｻｰ:REACTOR UNIT
  ox_object_ans[4] = 68 ; 評価対象番号ｱﾝｻｰ:TOP RACK UNIT
  ox_object_ans[5] = 69 ; 評価対象番号ｱﾝｻｰ:GAS UNIT
  ox_object_ans[6] = 70 ; 評価対象番号ｱﾝｻｰ:吊り具(1)下向き把持
  ox_object_ans[7] = 71 ; 評価対象番号ｱﾝｻｰ:吊り具(2)横向き把持
  ox_object_ans[8] = 72 ; 評価対象番号ｱﾝｻｰ:ﾏﾃﾊﾝ(Long)
  ox_object_ans[9] = 73 ; 評価対象番号ｱﾝｻｰ:ﾏﾃﾊﾝ(Short)
  ox_object_ans[10] = 74 ; 評価対象番号ｱﾝｻｰ:ﾅｯﾄﾗﾝﾅｰ1(M12)
  ox_object_ans[11] = 75 ; 評価対象番号ｱﾝｻｰ:ﾅｯﾄﾗﾝﾅｰ2(M12)
  ox_object_ans[12] = 76 ; 評価対象番号ｱﾝｻｰ:ﾅｯﾄﾗﾝﾅｰ(M20)
  ox_object_ans[13] = 77 ; 評価対象番号ｱﾝｻｰ:ﾛﾎﾞｯﾄ単体
;
  ox_unit_select[1] = 81 ; PG対応ワーク出力:RF UNIT
  ox_unit_select[2] = 82 ; PG対応ワーク出力:EXHAUST UNIT
  ox_unit_select[3] = 83 ; PG対応ワーク出力:REACTOR UNIT
  ox_unit_select[4] = 84 ; PG対応ワーク出力:TOP RACK UNIT
  ox_unit_select[5] = 85 ; PG対応ワーク出力:GAS UNIT
  ox_unit_select[6] = 86 ; PG対応ワーク出力:吊り具(1)下向き把持
  ox_unit_select[7] = 87 ; PG対応ワーク出力:吊り具(2)横向き把持
  ox_unit_select[8] = 88 ; PG対応ワーク出力:ﾏﾃﾊﾝ(Long)
  ox_unit_select[9] = 89 ; PG対応ワーク出力:ﾏﾃﾊﾝ(Short)
  ox_unit_select[10] = 90 ; PG対応ワーク出力:ﾅｯﾄﾗﾝﾅｰ1(M12)
  ox_unit_select[11] = 91 ; PG対応ワーク出力:ﾅｯﾄﾗﾝﾅｰ2(M12)
  ox_unit_select[12] = 92 ; PG対応ワーク出力:ﾅｯﾄﾗﾝﾅｰ(M20)  
  ox_unit_select[13] = 93 ; PG対応ワーク出力:ﾛﾎﾞｯﾄ単体  
;
  ox_close_atc[1] = 97 ; 吊り具ATC(1)ﾁｬｯｸ
  ox_open_atc[1] = 98 ; 吊り具ATC(1)ｱﾝﾁｬｯｸ
  ox_arrive_atc[1] = 99 ; 吊り具ATC(1)位置到着
  ox_close_atc[2] = 105 ; 吊り具ATC(2)ﾁｬｯｸ
  ox_open_atc[2] = 106 ; 吊り具ATC(2)ｱﾝﾁｬｯｸ
  ox_arrive_atc[2] = 107 ; 吊り具ATC(2)位置到着
;
  ox_close_atc[3] = 113 ; ﾏﾃﾊﾝATCﾁｬｯｸ
  ox_open_atc[3] = 114 ; ﾏﾃﾊﾝATCｱﾝﾁｬｯｸ
  ox_close_atc[4] = 113 ; ﾏﾃﾊﾝATCﾁｬｯｸ([3]と同じ)
  ox_open_atc[4] = 114 ; ﾏﾃﾊﾝATCｱﾝﾁｬｯｸ([3]と同じ)
  ox_arrive_atc[3] = 115 ; ﾏﾃﾊﾝATC(Long)位置到着
  ox_arrive_atc[4] = 116 ; ﾏﾃﾊﾝATC(Short)位置到着
;
  ox_close_atc[5] = 129 ; ﾅｯﾄﾗﾝﾅｰATCﾁｬｯｸ
  ox_open_atc[5] = 130 ; ﾅｯﾄﾗﾝﾅｰATCｱﾝﾁｬｯｸ
  ox_arrive_atc[5] = 131 ; ﾅｯﾄﾗﾝﾅｰ(M12_1)ATC位置到着
  ox_arrive_atc[6] = 132 ; ﾅｯﾄﾗﾝﾅｰ(M12_2)ATC位置到着
  ox_arrive_atc[7] = 133 ; ﾅｯﾄﾗﾝﾅｰ(M20)ATC位置到着
;
  ox_iai_home = 161 ; ﾛﾎﾞｼﾘﾝﾀﾞｰ原点に移動
  ox_iai_open = 162 ; ﾛﾎﾞｼﾘﾝﾀﾞｰ開き位置に移動
  ox_iai_close = 163 ; ﾛﾎﾞｼﾘﾝﾀﾞｰ閉じ位置に移動
  ox_iai_pow_on = 169 ; ﾛﾎﾞｼﾘﾝﾀﾞｰ電源ON指令(0729追加)
  ox_iai_pow_off = 170 ; ﾛﾎﾞｼﾘﾝﾀﾞｰ電源OFF指令(0729追加)
;
  ox_nat_vac_on[1] = 177 ; ﾅｯﾄﾗﾝﾅｰ吸着ON
  ox_nat_screw[1] = 178 ; ﾅｯﾄﾗﾝﾅｰﾄﾙｸ締め
  ox_nat_unscrew[1] = 179 ; ﾅｯﾄﾗﾝﾅｰゆるめ
  ox_nat_rotate[1] = 180 ; ﾅｯﾄﾗﾝﾅｰ空転
  ox_nat_vac_off[1] = 181 ; ﾅｯﾄﾗﾝﾅｰ吸着OFF
  ox_nat_stop[1] = 182 ; ﾅｯﾄﾗﾝﾅｰ動作停止
;
  ox_tightening = 201 ; ﾎﾞﾙﾄ締結動作中(PLCへ状態送信)
  ox_adsorbing = 202 ; ﾎﾞﾙﾄ吸着中(PLCへ状態送信)
;
  ox_rot_pos[1] = 205 ; ﾅｯﾄﾗﾝﾅｰ回転数受信選択_空転
  ox_rot_pos[2] = 206 ; ﾅｯﾄﾗﾝﾅｰ回転数受信選択_ﾄﾙｸ締め
  ox_rot_pos[3] = 207 ; ﾅｯﾄﾗﾝﾅｰ回転数受信選択_緩め
;
  ox_crn_paramset = 209 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ変更指令
  ox_crn_freemode = 210 ; ｱｼｽﾄｸﾚｰﾝ待機状態指令
  ox_crn_as_up = 211 ; ｱｼｽﾄｸﾚｰﾝｱｼｽﾄｱｯﾌﾟ指令
  ox_crn_as_down = 212 ; ｱｼｽﾄｸﾚｰﾝｱｼｽﾄﾀﾞｳﾝ指令
  ox_crn_as_dec = 213 ; ｱｼｽﾄｸﾚｰﾝｱｼｽﾄ減少指令
  ox_crn_pos[1] = 217 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ指示:空荷
  ox_crn_pos[2] = 218 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ指示:ﾏﾃﾊﾝ把持
  ox_crn_pos[3] = 219 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ指示:ﾕﾆｯﾄ把持
;
  ox_pussher_out[1] = 225 ; 水平ﾌﾟｯｼｬ(RF側)進行
  ox_pussher_ret[1] = 226 ; 水平ﾌﾟｯｼｬ(RF側)後退
  ox_pussher_out[2] = 228 ; 水平ﾌﾟｯｼｬ(EXHAUST側)進行
  ox_pussher_ret[2] = 229 ; 水平ﾌﾟｯｼｬ(EXHAUST側)後退
;
  ox_slider_out[1] = 233 ; ﾎﾞﾙﾄｽﾗｲﾀﾞｰ進行(取出位置)
  ox_slider_ret[1] = 234 ; ﾎﾞﾙﾄｽﾗｲﾀﾞｰ後退(供給位置)
;
  ox_arrive_sens = 241 ; ｾﾝｻｰ治具位置到着
;
  FOR .i = 0 TO 11 ; ﾅｯﾄﾗﾝﾅｰ回転数ｱﾝｻｰ12bit
    ox_nat_rpm_ans[.i+1] = 257+.i
  END
;
  ox_cam_light[1] = 273 ; 2D固定ｶﾒﾗ照明ON指令
;
  ox_il[1] = 289 ; 3Dﾋﾞｼﾞｮﾝ撮影用IL信号
  ox_il[2] = 290 ; 搬送完了通知用IL信号
  ox_il[3] = 291
  ox_il[4] = 292
  ox_il[5] = 293
  ox_il[6] = 294
  ox_il[7] = 295
  ox_il[8] = 296
;
  FOR .i = 0 TO 15 ; ｱﾌﾟﾘｴﾗｰ番号出力16bit
    ox_apperr_num[.i+1] = 481+.i
  END
;  
  FOR .i = 0 TO 9 ; ﾋﾞｼﾞｮﾝ補正IDｱﾝｻｰ10bit
    ox_vis_id_ans[.i+1] = 497+.i
  END
;
  FOR .i = 0 TO 6 ; 1軸ｹﾞｲﾝｱﾝｻｰ7bit
    ox_jt1gain_ans[.i+1] = 529+.i
  END
;
  FOR .i = 0 TO 6 ; 2軸ｹﾞｲﾝｱﾝｻｰ7bit
    ox_jt2gain_ans[.i+1] = 537+.i
  END
;
  FOR .i = 0 TO 6 ; 3軸ｹﾞｲﾝｱﾝｻｰ7bit
    ox_jt3gain_ans[.i+1] = 545+.i
  END
;
  FOR .i = 0 TO 6 ; 4軸ｹﾞｲﾝｱﾝｻｰ7bit
    ox_jt4gain_ans[.i+1] = 553+.i
  END
;
  FOR .i = 0 TO 6 ; 5軸ｹﾞｲﾝｱﾝｻｰ7bit
    ox_jt5gain_ans[.i+1] = 561+.i
  END
;
  FOR .i = 0 TO 6 ; 6軸ｹﾞｲﾝｱﾝｻｰ7bit
    ox_jt6gain_ans[.i+1] = 569+.i
  END
;
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝX座標出力(小数点前)14bit
    ox_vis_x_pos1[.i+1] = 769+.i
  END
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝX座標出力(小数点後)14bit
    ox_vis_x_pos2[.i+1] = 785+.i
  END
  ox_vis_x_pos_m = 800 ; ﾋﾞｼﾞｮﾝX座標出力ﾏｲﾅｽbit
;
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝY座標出力(小数点前)14bit
    ox_vis_y_pos1[.i+1] = 801+.i
  END
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝY座標出力(小数点後)14bit
    ox_vis_y_pos2[.i+1] = 817+.i
  END
  ox_vis_y_pos_m = 832 ; ﾋﾞｼﾞｮﾝY座標出力ﾏｲﾅｽbit
;
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝZ座標出力(小数点前)14bit
    ox_vis_z_pos1[.i+1] = 833+.i
  END
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝZ座標出力(小数点後)14bit
    ox_vis_z_pos2[.i+1] = 849+.i
  END
  ox_vis_z_pos_m = 864 ; ﾋﾞｼﾞｮﾝZ座標出力ﾏｲﾅｽbit  
;
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝO座標出力(小数点前)14bit
    ox_vis_o_pos1[.i+1] = 865+.i
  END
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝO座標出力(小数点後)14bit
    ox_vis_o_pos2[.i+1] = 881+.i
  END
  ox_vis_o_pos_m = 896 ; ﾋﾞｼﾞｮﾝO座標出力ﾏｲﾅｽbit    
;
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝA座標出力(小数点前)14bit
    ox_vis_a_pos1[.i+1] = 897+.i
  END
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝA座標出力(小数点後)14bit
    ox_vis_a_pos2[.i+1] = 913+.i
  END
  ox_vis_a_pos_m = 928 ; ﾋﾞｼﾞｮﾝA座標出力ﾏｲﾅｽbit  
;
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝT座標出力(小数点前)14bit
    ox_vis_t_pos1[.i+1] = 929+.i
  END
  FOR .i = 0 TO 13 ; ﾋﾞｼﾞｮﾝT座標出力(小数点後)14bit
    ox_vis_t_pos2[.i+1] = 945+.i
  END
  ox_vis_t_pos_m = 960 ; ﾋﾞｼﾞｮﾝT座標出力ﾏｲﾅｽbit        
;
;
;
;<< 入力信号 >>
  wx_motor_on = 1001 ; 外部ﾓｰﾀ電源ON
  wx_error_reset = 1002 ; 外部ｴﾗｰﾘｾｯﾄ
  wx_cyclestart = 1003 ; 外部ｻｲｸﾙｽﾀｰﾄ
  wx_pgreset = 1004 ; 外部ﾌﾟﾛｸﾞﾗﾑﾘｾｯﾄ
  wx_ext_hold = 1005 ; 外部停止
  wx_zpowoff = 1006 ; 外部ﾓｰﾀ電源OFF
  wx_home[1] = 1012 ; 原点復帰（全て元に戻す）
  wx_home[2] = 1013 ; 原点復帰（ﾏﾃﾊﾝまで元に戻す）
  wx_home[3] = 1014 ; 原点復帰（ﾕﾆｯﾄだけ元に戻す）  
;
  FOR .i = 0 TO 13 ; ﾛﾎﾞｯﾄPG番号選択14bit
    wx_pgno[.i+1] = 1017+.i
  END
;
  wx_restart = 1033 ; 評価再開指令
  wx_nmashi = 1034 ; 評価N増し指令
  wx_finish = 1035 ; 評価終了指令
;
  FOR .i = 0 TO 6 ; ﾁｪﾝｼﾞｹﾞｲﾝ位置指定ｱﾝｻｰ7bit
    wx_gain_pos_ans[.i+1] = 1041+.i
  END
;
  wx_trg_vis = 1049 ; ﾋﾞｼﾞｮﾝ座標PLC受信完了
;
  FOR .i = 0 TO 6 ; ﾋﾞｼﾞｮﾝﾛｸﾞ位置選択ｱﾝｻｰ7bit
    wx_vis_log_ans[.i+1] = 1050+.i
  END
;
  FOR .i = 0 TO 6 ; ﾋﾞｼﾞｮﾝ補正ID取得ｱﾝｻｰ7bit
    wx_vis_pos_ans[.i+1] = 1057+.i
  END
;
  wx_object[1] = 1065 ; 評価対象番号:RF UNIT
  wx_object[2] = 1066 ; 評価対象番号:EXHAUST UNIT
  wx_object[3] = 1067 ; 評価対象番号:REACTOR UNIT
  wx_object[4] = 1068 ; 評価対象番号:TOP RACK UNIT
  wx_object[5] = 1069 ; 評価対象番号:GAS UNIT
  wx_object[6] = 1070 ; 評価対象番号:吊り具(1)下向き把持
  wx_object[7] = 1071 ; 評価対象番号:吊り具(2)横向き把持
  wx_object[8] = 1072 ; 評価対象番号:ﾏﾃﾊﾝ(Long)
  wx_object[9] = 1073 ; 評価対象番号:ﾏﾃﾊﾝ(Short)
  wx_object[10] = 1074 ; 評価対象番号:ﾅｯﾄﾗﾝﾅｰ1(M12)
  wx_object[11] = 1075 ; 評価対象番号:ﾅｯﾄﾗﾝﾅｰ2(M12)
  wx_object[12] = 1076 ; 評価対象番号:ﾅｯﾄﾗﾝﾅｰ(M20)  
  wx_object[13] = 1077 ; 評価対象番号:ﾛﾎﾞｯﾄ単体  
;
  wx_table[1] = 1081 ; ﾕﾆｯﾄ置き台1選択
  wx_table[2] = 1082 ; ﾕﾆｯﾄ置き台2選択
;
  wx_close_atc[1] = 1097 ; 吊り具ATC(1)ﾁｬｯｸ
  wx_atc_open_ok[1] = 1098 ; 吊り具ATC(1)ｱﾝﾁｬｯｸ可
  wx_comp_cl_atc[1] = 1099 ; 吊り具ATC(1)ﾁｬｯｸ完了
  wx_comp_op_atc[1] = 1100 ; 吊り具ATC(1)ｱﾝﾁｬｯｸ完了
  wx_atc_onbase[1] = 1101 ; 吊り具ATC(1)置き台在籍
;
  wx_close_atc[2] = 1105 ; 吊り具ATC(2)ﾁｬｯｸ
  wx_atc_open_ok[2] = 1106 ; 吊り具ATC(2)ｱﾝﾁｬｯｸ可
  wx_comp_cl_atc[2] = 1107 ; 吊り具ATC(2)ﾁｬｯｸ完了
  wx_comp_op_atc[2] = 1108 ; 吊り具ATC(2)ｱﾝﾁｬｯｸ完了
  wx_atc_onbase[2] = 1101 ; 吊り具ATC(1)置き台在籍([1]と同じ)
;
  wx_close_atc[3] = 1113 ; ﾏﾃﾊﾝATC(Long)ﾁｬｯｸ
  wx_atc_open_ok[3] = 1114 ; ﾏﾃﾊﾝATC(Long)ｱﾝﾁｬｯｸ可
  wx_comp_cl_atc[3] = 1115 ; ﾏﾃﾊﾝATC(Long)ﾁｬｯｸ完了
  wx_comp_op_atc[3] = 1116 ; ﾏﾃﾊﾝATC(Long)ｱﾝﾁｬｯｸ完了
  wx_atc_onbase[3] = 1117 ; ﾏﾃﾊﾝATC(Long)置き台在籍
  wx_detect_work[3] = 1118 ; ﾏﾃﾊﾝATC(Long)ﾕﾆｯﾄ把持ｾﾝｻ
;
  wx_close_atc[4] = 1122 ; ﾏﾃﾊﾝATC(Short)ﾁｬｯｸ
  wx_atc_open_ok[4] = 1123 ; ﾏﾃﾊﾝATC(Short)ｱﾝﾁｬｯｸ可
  wx_comp_cl_atc[4] = 1124 ; ﾏﾃﾊﾝATC(Short)ﾁｬｯｸ完了
  wx_comp_op_atc[4] = 1125 ; ﾏﾃﾊﾝATC(Short)ｱﾝﾁｬｯｸ完了
  wx_atc_onbase[4] = 1126 ; ﾏﾃﾊﾝATC(Short)置き台在籍
  wx_detect_work[4] = 1127 ; ﾏﾃﾊﾝATC(Short)ﾕﾆｯﾄ把持ｾﾝｻ
;
  wx_close_atc[5] = 1129 ; ﾅｯﾄﾗﾝﾅｰATC1(M12_1)ﾁｬｯｸ
  wx_atc_open_ok[5] = 1130 ; ﾅｯﾄﾗﾝﾅｰATC1(M12_1)ｱﾝﾁｬｯｸ可
  wx_comp_cl_atc[5] = 1131 ; ﾅｯﾄﾗﾝﾅｰATC1(M12_1)ﾁｬｯｸ完了
  wx_comp_op_atc[5] = 1132 ; ﾅｯﾄﾗﾝﾅｰATC1(M12_1)ｱﾝﾁｬｯｸ完了
  wx_atc_onbase[5] = 1133 ; ﾅｯﾄﾗﾝﾅｰATC1(M12_1)置き台在籍  
;
  wx_close_atc[6] = 1137 ; ﾅｯﾄﾗﾝﾅｰATC2(M12_2)ﾁｬｯｸ
  wx_atc_open_ok[6] = 1138 ; ﾅｯﾄﾗﾝﾅｰATC2(M12_2)ｱﾝﾁｬｯｸ可
  wx_comp_cl_atc[6] = 1139 ; ﾅｯﾄﾗﾝﾅｰATC2(M12_2)ﾁｬｯｸ完了
  wx_comp_op_atc[6] = 1140 ; ﾅｯﾄﾗﾝﾅｰATC2(M12_2)ｱﾝﾁｬｯｸ完了
  wx_atc_onbase[6] = 1141 ; ﾅｯﾄﾗﾝﾅｰATC2(M12_2)置き台在籍    
;
  wx_close_atc[7] = 1145 ; ﾅｯﾄﾗﾝﾅｰATC(M20)ﾁｬｯｸ
  wx_atc_open_ok[7] = 1146 ; ﾅｯﾄﾗﾝﾅｰATC(M20)ｱﾝﾁｬｯｸ可
  wx_comp_cl_atc[7] = 1147 ; ﾅｯﾄﾗﾝﾅｰATC(M20)ﾁｬｯｸ完了
  wx_comp_op_atc[7] = 1148 ; ﾅｯﾄﾗﾝﾅｰATC(M20)ｱﾝﾁｬｯｸ完了
  wx_atc_onbase[7] = 1149 ; ﾅｯﾄﾗﾝﾅｰATC(M20)置き台在籍     
;
  wx_iai_home = 1161 ; ﾛﾎﾞｼﾘﾝﾀﾞｰ原点位置
  wx_iai_open = 1162 ; ﾛﾎﾞｼﾘﾝﾀﾞｰ開き位置
  wx_iai_close = 1163 ; ﾛﾎﾞｼﾘﾝﾀﾞｰ閉じ位置
  wx_iai_ready = 1164 ; ﾛﾎﾞｼﾘﾝﾀﾞｰREADY状態
;
  wx_nat_vac_on[1] = 1177 ; ﾅｯﾄﾗﾝﾅｰ吸着中
  wx_nat_trq_on[1] = 1178 ; ﾅｯﾄﾗﾝﾅｰﾄﾙｸ完了
  wx_nat_running[1] = 1179 ; ﾅｯﾄﾗﾝﾅｰ動作中
  wx_nat_ready = 1180 ; ﾅｯﾄﾗﾝﾅｰREADY状態
;
  wx_rot_pos_ans[1] = 1205 ; ﾅｯﾄﾗﾝﾅｰ回転数受信選択ｱﾝｻｰ_空転
  wx_rot_pos_ans[2] = 1206 ; ﾅｯﾄﾗﾝﾅｰ回転数受信選択ｱﾝｻｰ_ﾄﾙｸ締め
  wx_rot_pos_ans[3] = 1207 ; ﾅｯﾄﾗﾝﾅｰ回転数受信選択ｱﾝｻｰ_緩め  
;
  wx_crn_paramset = 1209 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ変更完了
  wx_crn_freemode = 1210 ; ｱｼｽﾄｸﾚｰﾝ待機状態変更完了
  wx_crn_as_up = 1211 ; ｱｼｽﾄｸﾚｰﾝｱｼｽﾄｱｯﾌﾟ指令完了
  wx_crn_as_down = 1212 ; ｱｼｽﾄｸﾚｰﾝｱｼｽﾄﾀﾞｳﾝ指令完了
  wx_crn_as_dec = 1213 ; ｱｼｽﾄｸﾚｰﾝｱｼｽﾄ減少指令完了
  wx_crn_pos[1] = 1217 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ状態:待機中
  wx_crn_pos[2] = 1218 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ状態:ﾊﾞﾗﾝｽ中
  wx_crn_pos[3] = 1219 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ状態:ﾎｰﾙﾄﾞ中  
  wx_crn_pos[4] = 1220 ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ状態:非常停止中  
  wx_crn_unit[1] = 1222 ; ｱｼｽﾄｸﾚｰﾝﾕﾆｯﾄ重量ｱｼｽﾄ中
;
  wx_pussher_out[1] = 1225 ; 水平ﾌﾟｯｼｬ(RF側)進行端
  wx_pussher_ret[1] = 1226 ; 水平ﾌﾟｯｼｬ(RF側)後退端
  wx_pussher_out[2] = 1228 ; 水平ﾌﾟｯｼｬ(EXHAUST側)進行端
  wx_pussher_ret[2] = 1229 ; 水平ﾌﾟｯｼｬ(EXHAUST側)後退端
;
  wx_slider_out[1] = 1233 ; ﾎﾞﾙﾄｽﾗｲﾀﾞｰ進行端(取出位置)
  wx_slider_ret[1] = 1234 ; ﾎﾞﾙﾄｽﾗｲﾀﾞｰ後退端(供給位置)
;  
  FOR .i = 0 TO 11 ; ﾅｯﾄﾗﾝﾅｰ回転数12bit
    wx_nat_rpm[.i+1] = 1257+.i
  END
  wx_nat_rpm_ok = 1272 ; ﾅｯﾄﾗﾝﾅｰ回転数照合OK    
;
  wx_cam_light[1] = 1273 ; 2D固定ｶﾒﾗ照明ON中  
;
  wx_il[1] = 1289 ; 3Dﾋﾞｼﾞｮﾝ撮影用IL信号
  wx_il[2] = 1290 ; 搬送完了通知用IL信号
  wx_il[3] = 1291
  wx_il[4] = 1292
  wx_il[5] = 1293
  wx_il[6] = 1294
  wx_il[7] = 1295
  wx_il[8] = 1296
;  
  FOR .i = 0 TO 9 ; ﾋﾞｼﾞｮﾝ補正ID10bit
    wx_vis_id[.i+1] = 1497+.i
  END
  wx_vis_id_ok = 1512 ; ﾋﾞｼﾞｮﾝ補正ID照合OK  
;  
  FOR .i = 0 TO 6 ; 1軸ｹﾞｲﾝ7bit
    wx_jt1gain[.i+1] = 1529+.i
  END
;
  FOR .i = 0 TO 6 ; 2軸ｹﾞｲﾝ7bit
    wx_jt2gain[.i+1] = 1537+.i
  END
;
  FOR .i = 0 TO 6 ; 3軸ｹﾞｲﾝ7bit
    wx_jt3gain[.i+1] = 1545+.i
  END
;
  FOR .i = 0 TO 6 ; 4軸ｹﾞｲﾝ7bit
    wx_jt4gain[.i+1] = 1553+.i
  END
;
  FOR .i = 0 TO 6 ; 5軸ｹﾞｲﾝ7bit
    wx_jt5gain[.i+1] = 1561+.i
  END
;
  FOR .i = 0 TO 6 ; 6軸ｹﾞｲﾝ7bit
    wx_jt6gain[.i+1] = 1569+.i
  END
  wx_gain_ok = 1576 ; ﾁｪﾝｼﾞｹﾞｲﾝ値照合OK
;
;<< 内部信号 >>
;
.END
.PROGRAM z.init_real() #45; 変数設定
; /======================================================================/
; FUNCTION: 変数設定
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.
; /======================================================================/
;
  rb_no = 1 ; RF側はRB1
;
;<< ｱﾌﾟﾘ >>
  sys_timeout = 30 ; ﾀｲﾑｱｳﾄ時間
  sil_timeout = 60 ; ｼﾘﾝﾀﾞ操作ﾀｲﾑｱｳﾄ時間
  pulse_tim = 0.5 ; ﾊﾟﾙｽ信号出力時間
  stab_tim = 1 ; 防振 TIMER
;
;<<ﾂｰﾙ変数>>
  POINT no_tool = TRANS(0,0,235,0,0,0)
  POINT tool_tsuri_up = TRANS(0,-460,348,0,0,0)
  POINT tool_tsuri_side = TRANS(-30.13,54.56,711.6,-60,90,-90)
  POINT tool_long = TRANS(395,-610,935,-180,180,0)
  POINT tool_short = TRANS(395,-610,735,-180,180,0)
  POINT tool_nat_m12 = TRANS(300,0,504,-90,0,0)
  POINT grab_tool = no_tool ; ﾏｽﾀｰ計測時のﾂｰﾙ(固定)
;
;<< 負荷質量kg >>
  kg_max = 100 ; 最大可搬重量
;把持無し
  kg_none = 30
  gx_none = 11.5
  gy_none = -26.6
  gz_none = 149.6
;吊り具
  kg_tsuri = 100
  gx_tsuri_up = 29.8
  gy_tsuri_up = -183.6
  gz_tsuri_up = 226.5
  gx_tsuri_side = 32.6
  gy_tsuri_side = 81.3
  gz_tsuri_side = 397.4
;ﾅｯﾄﾗﾝﾅｰ
  kg_nat_m12 = 60.6
  gx_nat_m12 = -30.2
  gy_nat_m12 = -28.4
  gz_nat_m12 = 208
;ﾏﾃﾊﾝ、ﾕﾆｯﾄは可搬重量以上のため設定しない
;
;<< 3Dﾋﾞｼﾞｮﾝ >>
;Mech-mind IPaddress
  mm_ip[1] = 192
  mm_ip[2] = 168
  mm_ip[3] = 0
  mm_ip[4] = 13
  mm_port[1] = 50000
;↓ﾛｸﾞｶｳﾝﾀ変数
  IF EXISTREAL("cnt5[101]")==0 THEN
    cnt5[101] = 1
  END
  IF EXISTREAL("cnt5[102]")==0 THEN
    cnt5[102] = 1
  END
  IF EXISTREAL("cnt5[103]")==0 THEN
    cnt5[103] = 1
  END
  IF EXISTREAL("cnt5[104]")==0 THEN
    cnt5[104] = 1
  END
  IF EXISTREAL("cnt5[105]")==0 THEN
    cnt5[105] = 1
  END
  IF EXISTREAL("cnt3[201]")==0 THEN
    cnt3[201] = 1
  END
  IF EXISTREAL("cnt3[202]")==0 THEN
    cnt3[202] = 1
  END
  IF EXISTREAL("cnt3[203]")==0 THEN
    cnt3[203] = 1
  END
  IF EXISTREAL("cnt3[204]")==0 THEN
    cnt3[204] = 1
  END
  IF EXISTREAL("cnt3[205]")==0 THEN
    cnt3[205] = 1
  END
  IF EXISTREAL("cnt3[301]")==0 THEN
    cnt3[301] = 1
  END
  IF EXISTREAL("cnt3[302]")==0 THEN
    cnt3[302] = 1
  END
  IF EXISTREAL("cnt3[303]")==0 THEN
    cnt3[303] = 1
  END
  IF EXISTREAL("cnt3[304]")==0 THEN
    cnt3[304] = 1
  END
  IF EXISTREAL("cnt3[305]")==0 THEN
    cnt3[305] = 1
  END
  IF EXISTREAL("cnt3[401]")==0 THEN
    cnt3[401] = 1
  END
  IF EXISTREAL("cnt3[402]")==0 THEN
    cnt3[402] = 1
  END
  IF EXISTREAL("cnt3[403]")==0 THEN
    cnt3[403] = 1
  END
  IF EXISTREAL("cnt3[404]")==0 THEN
    cnt3[404] = 1
  END
  IF EXISTREAL("cnt3[405]")==0 THEN
    cnt3[405] = 1
  END
  IF EXISTREAL("cnt3[501]")==0 THEN
    cnt3[501] = 1
  END
  IF EXISTREAL("cnt3[502]")==0 THEN
    cnt3[502] = 1
  END
  IF EXISTREAL("cnt3[503]")==0 THEN
    cnt3[503] = 1
  END
  IF EXISTREAL("cnt3[504]")==0 THEN
    cnt3[504] = 1
  END
  IF EXISTREAL("cnt3[505]")==0 THEN
    cnt3[505] = 1
  END
  IF EXISTREAL("cnt1[11]")==0 THEN
    cnt1[11] = 1
  END    ;
  IF EXISTREAL("cnt1[12]")==0 THEN
    cnt1[12] = 1
  END    ;
  IF EXISTREAL("cnt1[13]")==0 THEN
    cnt1[13] = 1
  END    ;
  IF EXISTREAL("cnt1[14]")==0 THEN
    cnt1[14] = 1
  END    ;
  IF EXISTREAL("cnt1[15]")==0 THEN
    cnt1[15] = 1
  END    ;
  IF EXISTREAL("cnt1[16]")==0 THEN
    cnt1[16] = 1
  END
  IF EXISTREAL("cnt1[17]")==0 THEN
    cnt1[17] = 1
  END
  IF EXISTREAL("cnt1[18]")==0 THEN
    cnt1[18] = 1
  END
  IF EXISTREAL("cnt1[19]")==0 THEN
    cnt1[19] = 1
  END
  IF EXISTREAL("cnt1[20]")==0 THEN
    cnt1[20] = 1
  END
  IF EXISTREAL("cnt1[21]")==0 THEN
    cnt1[21] = 1
  END
  IF EXISTREAL("cnt1[22]")==0 THEN
    cnt1[22] = 1
  END
  IF EXISTREAL("cnt1[1000]")==0 THEN
    cnt1[1000] = 1
  END
;
;<< 無把持ﾊﾟﾗﾒｰﾀ >>
  max.sp = 100 ; 最大速度(%)
  max.acu = 1 ; 最小精度
  max.acc = 100 ; 最大加速度
  max.dec = 100 ; 最大減速度
;
;** 吊り具ﾊﾟﾗﾒｰﾀ **
  tsuri_xy_sp[1] = 30 ; 吊り具のみの水平速度(mon) (0801変更)
  tsuri_z_sp[1] = 200 ; 吊り治具の上下速度(mm/s) (0804変更)
  tsuri_acc = 0.5 ; 吊り具のみの加速度
  tsuri_dec = 0.5 ; 吊り具のみの減速度
;
;** ﾏﾃﾊﾝﾊﾟﾗﾒｰﾀ **
  hand_xy_sp[1] = 500 ; ﾏﾃﾊﾝlong上持ちの水平速度(mm/s)
  hand_z_sp[1] = 100 ; ﾏﾃﾊﾝLong上持ちの上下速度(mm/s)
  hand_xy_sp[2] = 500 ; ﾏﾃﾊﾝShort上持ちの水平速度(mm/s)
  hand_z_sp[2] = 100 ; ﾏﾃﾊﾝShort上持ちの上下速度(mm/s)
  hand_acc[1] = 0.2 ; ﾏﾃﾊﾝlongの上下加速度
  hand_dec[1] = 0.2 ; ﾏﾃﾊﾝlongの上下減速度
  hand_acc[2] = 0.2 ; ﾏﾃﾊﾝshortの上下加速度
  hand_dec[2] = 0.2 ; ﾏﾃﾊﾝshortの上下減速度
  hand_xy_acc[1] = 5 ; ﾏﾃﾊﾝlongの水平加速度
  hand_xy_dec[1] = 5 ; ﾏﾃﾊﾝlongの水平減速度
  hand_xy_acc[2] = 5 ; ﾏﾃﾊﾝshortの水平加速度
  hand_xy_dec[2] = 5 ; ﾏﾃﾊﾝshortの水平減速度
;
;** ﾅｯﾄﾗﾝﾅﾊﾟﾗﾒｰﾀ **
  pick_bolt_sp[1] = 1.5 ; ﾎﾞﾙﾄ吸着時速度(mm/s)(0805変更)
  screw_apr_sp[1] = 50 ; 締結開始位置まで進む速度(mm/s)
  nat_sp[1] = 100 ; ﾅｯﾄﾗﾝﾅｰ移動速度(%)
  nat_acc = 100 ; ﾅｯﾄﾗﾝﾅｰ加速度
  nat_dec = 100 ; ﾅｯﾄﾗﾝﾅｰ減速度
  m12_pitch = 1.75 ; M12ねじﾋﾟｯﾁ
  m20_pitch = 2.5 ; M20ねじﾋﾟｯﾁ
;
;** その他ﾊﾟﾗﾒｰﾀ **
  atc_chk_sp[1] = 100 ; ATCﾁｬｯｸ速度(mm/s)
  pick_work_sp[1] = 2 ; ﾕﾆｯﾄにﾊﾝﾄﾞのﾋﾟﾝ入れる速度(mm/s)
  set_work_sp[1] = 1 ; ﾕﾆｯﾄからﾊﾝﾄﾞのﾋﾟﾝ抜くときの速度(mm/s)
;
;** RFﾊﾟﾗﾒｰﾀ **
  work_xy_sp[1] = 250 ; RFのｱｼｽﾄ中水平速度(mm/s)
  work_z_sp[1] = 30 ; RFのｱｼｽﾄ中上下速度(mm/s)
  work_acc[1] = 0.05 ; RF把持中の上下移動加速度
  work_dec[1] = 0.05 ; RF把持中の上下移動減速度
  work_xy_acc[1] = 0.5 ; RF把持中の水平移動加速度
  work_xy_dec[1] = 0.5 ; RF把持中の水平移動減速度
  m12bolt_pc[1] = 8 ; ﾎﾞﾙﾄ締結本数
  m12bolt_length[1] = 30 ; ﾎﾞﾙﾄ長さ
;
;** REACTORﾊﾟﾗﾒｰﾀ **
  work_xy_sp[3] = 250 ; REACTORのｱｼｽﾄ中水平速度(mm/s)
  work_z_sp[3] = 45 ; REACTORのｱｼｽﾄ中上下速度(mm/s)
  work_acc[3] = 0.05 ; REACTOR把持中の上下移動加速度 0905変更前
  work_dec[3] = 0.05 ; REACTOR把持中の上下移動減速度 0905変更前
  work_xy_acc[3] = 2 ; REACTOR把持中の水平移動加速度0829 0905変更前
  work_xy_dec[3] = 2 ; REACTOR把持中の水平移動減速度0829 0905変更前
  m12bolt_pc[3] = 8 ; ﾎﾞﾙﾄ締結本数(手前側の8本)
  m12bolt_length[3] = 30 ; ﾎﾞﾙﾄ長さ
;
;** TOPRACKﾊﾟﾗﾒｰﾀ **
  m12bolt_pc[4] = 2 ; ﾎﾞﾙﾄ締結本数(手前側の2本)
  m12bolt_length[4] = 30 ; ﾎﾞﾙﾄ長さ
;
  RETURN
;
.END

.PROGRAM md_chk_clamp(.flg) #8; ﾛﾎﾞｯﾄ把持確認
; /======================================================================/
; FUNCTION: ﾛﾎﾞｯﾄ把持確認。ロボットが何か持っていないかを確認するためのPG
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  .flg = 0
;
  FOR .i = 1 TO 7
    cl[.i] = OFF ; 初期化
  END
;
  FOR .i = 1 TO 7
    IF SIG(wx_close_atc[.i]) AND SIG(-wx_atc_onbase[.i]) THEN ; 各ATCのﾁｬｯｸ信号/置き台在籍無し
      cl[.i] = ON
    END
  END
;
;cl[1] : 吊り具ATC(1)ﾁｬｯｸ中(下向き)
;cl[2] : 吊り具ATC(2)ﾁｬｯｸ中(横向き)
;cl[3] : ﾏﾃﾊﾝATC(Long)ﾁｬｯｸ中
;cl[4] : ﾏﾃﾊﾝATC(Short)ﾁｬｯｸ中
;cl[5] : ﾅｯﾄﾗﾝﾅｰ1ATC(M12)ﾁｬｯｸ中→RB1のみ
;cl[6] : ﾅｯﾄﾗﾝﾅｰ2ATC(M12)ﾁｬｯｸ中→RB2のみ
;cl[7] : ﾅｯﾄﾗﾝﾅｰATC(M20)ﾁｬｯｸ中→RB2のみ
;
;ﾕﾆｯﾄまで把持している場合
  IF cl[1]==ON OR cl[2]==ON THEN ; 吊り具どちらか把持中
    IF cl[3]==ON OR cl[4]==ON THEN ; ﾏﾃﾊﾝどちらか把持中
      IF SIG(wx_iai_close) AND SIG(-wx_iai_open) AND SIG(-wx_iai_home) THEN ; IAI閉じ位置
        IF SIG(wx_detect_work[3]) OR SIG(wx_crn_unit[1]) THEN ; ﾕﾆｯﾄ把持ｾﾝｻON/ｱｼｽﾄｸﾚｰﾝﾕﾆｯﾄ重量ｱｼｽﾄ中
          .flg = 5 ;●ﾏﾃﾊﾝLongでﾕﾆｯﾄ把持中
          GOTO end
        END
        IF SIG(wx_detect_work[4]) OR SIG(wx_crn_unit[1]) THEN ; ﾕﾆｯﾄ把持ｾﾝｻON
          .flg = 6 ;●ﾏﾃﾊﾝShortでﾕﾆｯﾄ把持中
          GOTO end
        END
      END
    END
  END
;
;ﾏﾃﾊﾝまで把持している場合
  IF cl[1]==ON OR cl[2]==ON THEN ; 吊り具どちらか把持中
    IF cl[3]==ON OR cl[4]==ON THEN ; ﾏﾃﾊﾝどちらか把持中
      IF (SIG(wx_iai_open) OR SIG(wx_iai_home)) AND SIG(-wx_iai_close) THEN ; IAI開きor原点位置
        IF cl[3]==ON AND SIG(-wx_detect_work[3]) AND SIG(wx_crn_unit[1]) THEN ; ﾏﾃﾊﾝlong/  WORK検出ｾﾝｻON
          .flg = 3 ; ●ﾏﾃﾊﾝlong把持中  
          GOTO end
        END
        IF cl[4]==ON AND SIG(-wx_detect_work[4]) AND SIG(wx_crn_unit[1]) THEN ; ﾏﾃﾊﾝshort/  WORK検出ｾﾝｻON
          .flg = 4 ; ●ﾏﾃﾊﾝshort把持中  
          GOTO end
        END
      END
    END
  END
;
;吊り具まで把持している場合
  IF cl[1]==ON THEN ; 吊り具上で把持中
    IF cl[3]==OFF AND cl[4]==OFF THEN ; ﾏﾃﾊﾝどちらも把持していない
      .flg = 1 ;●吊り具上で把持中
      GOTO end
    END
  END
  IF cl[2]==ON THEN ; 吊り具横で把持中
    IF cl[3]==OFF AND cl[4]==OFF THEN ; ﾏﾃﾊﾝどちらも把持していない
      .flg = 2 ;●吊り具横で把持中
      GOTO end
    END
  END
;
;ﾅｯﾄﾗﾝﾅｰを把持している場合  
  IF cl[5]==ON THEN ; ﾅｯﾄﾗﾝﾅｰどれか把持中
    .flg = 7 ;●ﾅｯﾄﾗﾝﾅｰM12_1把持中
    GOTO end
  END
  IF cl[6]==ON THEN ; ﾅｯﾄﾗﾝﾅｰどれか把持中
    .flg = 8 ;●ﾅｯﾄﾗﾝﾅｰM12_2把持中
    GOTO end
  END
  IF cl[7]==ON THEN ; ﾅｯﾄﾗﾝﾅｰどれか把持中
    .flg = 9 ;●ﾅｯﾄﾗﾝﾅｰM20把持中
    GOTO end
  END
;
end:
;
  RETURN
;
.END


.PROGRAM md_tool_init() #3; ﾂｰﾙ初期化処理
; /======================================================================/
; FUNCTION: ﾂｰﾙ初期化処理
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
;<< 無把持でTOOL初期化する処理 >>
  IF SIG(-wx_close_atc[1]) AND SIG(-wx_close_atc[2]) AND SIG(wx_atc_onbase[1]) AND SIG(wx_atc_onbase[2]) THEN ; 吊り具ｱﾝﾁｬｯｸ中/置き台在籍あり
    IF SIG(-wx_close_atc[3]) AND SIG(-wx_close_atc[4]) AND SIG(wx_atc_onbase[3]) AND SIG(wx_atc_onbase[4]) THEN ; ﾏﾃﾊﾝｱﾝﾁｬｯｸ中/置き台在籍あり
      IF rb_no==1 THEN ; RB1
        IF SIG(-wx_close_atc[5]) AND SIG(wx_atc_onbase[5]) THEN ; ﾅｯﾄﾗﾝﾅｰｱﾝﾁｬｯｸ中/置き台在籍あり
          TOOL no_tool
          WEIGHT kg_none,gx_none,gy_none,gz_none; 把持無し_重量/重心  
        ELSE
          HALT ; 異常
;ELSE-RB1ﾅｯﾄﾗﾝﾅｰ把持?
        END
      ELSE ; RB2
        IF SIG(-wx_close_atc[6]) AND SIG(-wx_close_atc[7]) AND SIG(wx_atc_onbase[6]) AND SIG(wx_atc_onbase[7]) THEN ; ﾅｯﾄﾗﾝﾅｰｱﾝﾁｬｯｸ中/置き台在籍あり
          TOOL no_tool
          WEIGHT kg_none,gx_none,gy_none,gz_none; 把持無し_重量/重心  
        ELSE
          HALT ; 異常
;ELSE-RB2ﾅｯﾄﾗﾝﾅｰ把持?
        END
      END
    ELSE
      HALT ; 異常
;ELSE-ﾏﾃﾊﾝ把持?
    END
  ELSE
    HALT ; 異常
;ELSE-吊り具把持?
  END
;
.END



;***********************************************************
;ビジョン処理関連
;***********************************************************

.PROGRAM mk_vision_combi(.id,.point) #4; ﾋﾞｼﾞｮﾝ処理組合せ確認
;
  v.err = 0
  v.error_code[.id] = 0
  .retry = 0
;
  $v.message[.id] = ""; initialize
  .ext_id = EXISTREAL("vc_master_comp["+$ENCODE(/L,.id)+"]")
  IF .ext_id==TRUE THEN
    CALL vcompatible5(.id)
    .ret_id = vc_master_comp[.id]
  ELSE
    .ret_id = FALSE
  END
  IF .ret_id==TRUE THEN
; check regist master
    .have_num = 0;
    FOR .i = 1 TO vc_multi_type[.id]+2
      IF .i==1 THEN
        .$chk_master = "vc_master_comp["+$ENCODE(/L,.id)+"]"
      ELSE
        .$chk_master = "vc_master_comp"+$ENCODE(/L,.i)+"["+$ENCODE(/L,.id)+"]"
      END
;      .ret_master = EXISTREAL(.$chk_master); check master measurement
      .ret_master = TRUE
      IF .ret_master==TRUE THEN
        CASE .i OF
         VALUE 1: ;
          .h = vc_have[.id]
         VALUE 2: ;
          .h = vc_have2[.id]
         VALUE 3: ;
          .h = vc_have3[.id]
         VALUE 4: ;
          .h = vc_have4[.id]
         VALUE 5: ;
          .h = vc_have5[.id]
        END
        IF .h==TRUE THEN
          .have_num = .have_num+1
        END
      ELSE
        v.err = 71; disable correction
        v.error_code[.id] = 71
        CALL vget_error_mess(v.err,$v.err_message); error message
        $v.message[.id] = $v.err_message ; save error message
        RETURN
      END
    END
    IF vc_multi_type[.id]==2 THEN
      IF .have_num!=2 THEN
        v.err = 73 ; have type error
        v.error_code[.id] = 73
        CALL vget_error_mess(v.err,$v.err_message); error message
        $v.message[.id] = $v.err_message ; save error message
        RETURN
      END
    END
; get parameters
    CASE .point OF
     VALUE 1:
      .cam_no = vc_cam[.id]; cam no
      .mes_no = vc_mes[.id]; measure no
      .have = vc_have[.id]; have type
      .use_cam = vc_use_cam[.id];use cam
      IF .use_cam==FALSE THEN
        .kari_mes_no = vc_kari_mes[.id]
      END
     VALUE 2:
      .cam_no = vc_cam2[.id]; cam no
      .mes_no = vc_mes2[.id]; measure no
      .have = vc_have2[.id]; have type
      .use_cam = vc_use_cam2[.id];use cam
      IF .use_cam==FALSE THEN
        .kari_mes_no = vc_kari_mes2[.id]
      END
     VALUE 3:
      .cam_no = vc_cam3[.id]; cam no
      .mes_no = vc_mes3[.id]; measure no
      .have = vc_have3[.id]; have type
      .use_cam = vc_use_cam3[.id];use cam
      IF .use_cam==FALSE THEN
        .kari_mes_no = vc_kari_mes3[.id]
      END
     VALUE 4:
      .cam_no = vc_cam4[.id]; cam no
      .mes_no = vc_mes4[.id]; measure no
      .have = vc_have4[.id]; have type
      .use_cam = vc_use_cam4[.id];use cam
      IF .use_cam==FALSE THEN
        .kari_mes_no = vc_kari_mes4[.id]
      END
     VALUE 5:
      .cam_no = vc_cam5[.id]; cam no
      .mes_no = vc_mes5[.id]; measure no
      .have = vc_have5[.id]; have type
      .use_cam = vc_use_cam5[.id];use cam
      IF .use_cam==FALSE THEN
        .kari_mes_no = vc_kari_mes5[.id]
      END
    END
retry:
    v.err = 0
    v.error_code[.id] = 0
    .retry = .retry+1 ;リトライ追加
    IF .use_cam==TRUE THEN
      CALL vision_exe(.cam_no,.mes_no,.have); execute vision measument
    ELSE
      CALL mm_vision_exe(.mes_no,.have,.kari_mes_no,.id,.point)
    END
    CALL vchk_ignore_res(.id,.ignore_res); result no ignore check
    v.work_num[.id] = v.num ; save result number
    v.error_code[.id] = v.err ; save error code
    IF .retry>3 THEN ;リトライNG
      v.err = 104 ; retry NG
      v.error_code[.id] = 104
      CALL vget_error_mess(v.err,$v.err_message); error message
      $v.message[.id] = $v.err_message ; save error message
      RETURN
    END
    IF v.err<>0 GOTO retry     ;リトライ
retry_err:
    FOR .i = 1 TO 1
      IF .ignore_res==OFF THEN
        .result_no = v.result_no[.mes_no,.i]; save result no
      ELSE
        .result_no = 1; set result no to 1
      END
      IF .use_cam==TRUE THEN
        CASE .point OF
         VALUE 1:
          v.work_res[.id,.i] = .result_no
          POINT v.workbc[.id,.i] = vwork[.mes_no,.i]; save workpiece position(base)
          POINT v.workvc[.id,.i] = vision[.i]; save workpiece position(vision)
         VALUE 2:
          v.work_res2[.id,.i] = .result_no
          POINT v.workbc2[.id,.i] = vwork[.mes_no,.i]; save workpiece position(base)
          POINT v.workvc2[.id,.i] = vision[.i]; save workpiece position(vision)
         VALUE 3:
          v.work_res3[.id,.i] = .result_no
          POINT v.workbc3[.id,.i] = vwork[.mes_no,.i]; save workpiece position(base)
          POINT v.workvc3[.id,.i] = vision[.i]; save workpiece position(vision)
         VALUE 4:
          v.work_res4[.id,.i] = .result_no
          POINT v.workbc4[.id,.i] = vwork[.mes_no,.i]; save workpiece position(base)
          POINT v.workvc4[.id,.i] = vision[.i]; save workpiece position(vision)
         VALUE 5:
          v.work_res5[.id,.i] = .result_no
          POINT v.workbc5[.id,.i] = vwork[.mes_no,.i]; save workpiece position(base)
          POINT v.workvc5[.id,.i] = vision[.i]; save workpiece position(vision)         
        END
      ELSE
        CASE .point OF
         VALUE 1:
          v.work_res[.id,.i] = 1
          POINT v.workbc[.id,.i] = mwork[.mes_no,.i]; save workpiece position(base)
         VALUE 2:
          v.work_res2[.id,.i] = 1
          POINT v.workbc2[.id,.i] = mwork[.mes_no,.i]; save workpiece position(base)
         VALUE 3:
          v.work_res3[.id,.i] = 1
          POINT v.workbc3[.id,.i] = mwork[.mes_no,.i]; save workpiece position(base)
         VALUE 4:
          v.work_res4[.id,.i] = 1
          POINT v.workbc4[.id,.i] = mwork[.mes_no,.i]; save workpiece position(base)
         VALUE 5:
          v.work_res5[.id,.i] = 1
          POINT v.workbc5[.id,.i] = mwork[.mes_no,.i]; save workpiece position(base)
        END
      END
      CALL vmaster_check(.id,v.work_res[.id,.i],.ret); check master registration(current point)
      IF .ret==TRUE AND .i==1 THEN
        IF vc_multi_type[.id]+2==.point THEN ; correction only last measurement
          FOR .j = 1 TO vc_pos_max[.id,v.work_res[.id,.i]]
            CASE vc_multi_type[.id] OF
             VALUE 0:
              CALL vcorrect_2p(&vpick_master[.id,v.work_res[.id,.i],.j],.id,&v.pos[.id,.i,.j]); make corrected position
;             VALUE 1:
;              CALL vcorrect_3p(&vpick_master[.id,v.work_res[.id,.i],.j],.id,&v.pos[.id,.i,.j]); make corrected position
             VALUE 1:
              CALL mcorrect_3p(&vpick_master[.id,v.work_res[.id,.i],.j],.id,&v.pos[.id,.i,.j]); make corrected position
              CALL mm_tolerance_ch
             VALUE 2:
              CALL vcorrect_4p(&vpick_master[.id,v.work_res[.id,.i],.j],.id,&v.pos[.id,.i,.j]); make corrected position
              CALL mm_tolerance_ch
             VALUE 3:
              CALL mcorrect_5p(&vpick_master[.id,v.work_res[.id,.i],.j],.id,&v.pos[.id,.i,.j]); make corrected position             
              CALL mm_tolerance_ch
            END
;            POINT v.diffteach[.id,.i,.j] = -vpick_master[.id,v.work_res[.id,.i],.j]+v.pos[.id,.i,.j]; save difference teach and correct
          END
          .dxb = DEXT(v.workbc[.id,.i],1)-DEXT(v_master[.id,v.work_res[.id,.i]],1)
          .dyb = DEXT(v.workbc[.id,.i],2)-DEXT(v_master[.id,v.work_res[.id,.i]],2)
          .dzb = DEXT(v.workbc[.id,.i],3)-DEXT(v_master[.id,v.work_res[.id,.i]],3)
          .aw = DEXT(v.workbc[.id,.i],4)+DEXT(v.workbc[.id,.i],6)
          .am = DEXT(v_master[.id,v.work_res[.id,.i]],4)+DEXT(v_master[.id,v.work_res[.id,.i]],6)
          .dab = .aw-.am
        END
      ELSE
        v.err = 71; disable correction    
        v.error_code[.id] = 71
      END
    END
  ELSE
err_stop:
    v.err = 71; not regist master
    v.error_code[.id] = 71
  END
  IF v.error_code[.id]!=0 THEN ; error check
    CALL vget_error_mess(v.err,$v.err_message); error message
    $v.message[.id] = $v.err_message ; save error message
  END
.END

.PROGRAM md_recv_vis_id(.pos,.id) #15; ﾋﾞｼﾞｮﾝ補正ID受信
; /======================================================================/
; FUNCTION: ﾋﾞｼﾞｮﾝ補正ID受信
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
;[.pos]
; 1:ﾕﾆｯﾄ組み込み
; 2:ﾕﾆｯﾄ取り出し
; 3:ﾈｼﾞ締め
; 11:検査1
; 12:検査2
; 13:検査3
; 21:ﾕﾆｯﾄ取り外し(戻し動作時)
; 22:ﾕﾆｯﾄ戻し(戻し動作時)
; ※検査の番号は仮31:RBの歪み計測
;
; 32:吊り具の歪み計測
; 33:マテハンlongの歪み計測1
; 34:マテハンshortの歪み計測1
; 35:ナットランナーM12の歪み計測
; 36:ナットランナーM20の歪み計測
; 24:マテハンlongの歪み計測2
; 25:マテハンlongの歪み計測3
; 26:マテハンlongの歪み計測4
; 27:マテハンshortの歪み計測2
; 28:マテハンshortの歪み計測3
; 29:マテハンshortの歪み計測4
; 41:M12ボルト位置計測
; 42:M20ボルト位置計測
;
;.id→返値
;
  BITS ox_vis_pos[1],7 = .pos
  TIMER 1 = 0
  DO
    IF TIMER(1)>sys_timeout THEN
      CALL z.app_error("ﾋﾞｼﾞｮﾝ補正ID位置番号受信ｴﾗｰ")
    END
  UNTIL BITS(wx_vis_pos_ans[1],7)==.pos ; 照合OK  
;  
  TIMER 1 = 0
  DO
    .id = BITS(wx_vis_id[1],10) ; ﾋﾞｼﾞｮﾝID受信
    TWAIT ascycle
    BITS ox_vis_id_ans[1],10 = .id
;
    IF TIMER(1)>sys_timeout THEN
      CALL z.app_error("ﾋﾞｼﾞｮﾝ補正ID受信ｴﾗｰ")
    END
  UNTIL SIG(wx_vis_id_ok) ; 照合OK
;
;ﾘｾｯﾄ
  BITS ox_vis_pos[1],7 = 0
;
  RETURN
;
.END

.PROGRAM mm_tolerance(.id) #2; ﾋﾞｼﾞｮﾝ補正結果の範囲ﾁｪｯｸ
; /======================================================================/
; FUNCTION: ﾋﾞｼﾞｮﾝ補正結果の範囲ﾁｪｯｸ
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
;mk_vision_combiより前に呼び出す
;
;<<初期化>>
  tole_pos = 5 ; XYZの上下限
  tole_ang = 2 ; RX,RY,RZの上下限
;
  CASE .id OF
   VALUE 101: ; RF置き
    tole_pos = 5
    tole_ang = 2
   VALUE 201: ; RF取り
    tole_pos = 5
    tole_ang = 2
   VALUE 301: ; RFﾈｼﾞ締め
    tole_pos = 5
    tole_ang = 2
   VALUE 401: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 501: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 102: ; EXHAUST置き
    tole_pos = 5
    tole_ang = 2
   VALUE 202: ; EXHAUST取り
    tole_pos = 5
    tole_ang = 2
   VALUE 302: ; EXHAUSTﾈｼﾞ締め
    tole_pos = 5
    tole_ang = 2
   VALUE 402: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 502: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 103: ; REACTOR置き
    tole_pos = 5
    tole_ang = 2
   VALUE 203: ; REACTOR取り
    tole_pos = 10 ; (0825変更_REACはｽﾞﾚ量大きい)
    tole_ang = 2
   VALUE 303: ; REACTORﾈｼﾞ締め
    tole_pos = 5
    tole_ang = 2
   VALUE 403: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 503: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 104: ; TOPRACK置き
    tole_pos = 5
    tole_ang = 2
   VALUE 204: ; TOPRACK取り
    tole_pos = 10 ; (1022変更_置き位置ｽﾞﾚ大きくなる為)
    tole_ang = 2
   VALUE 304: ; TOPRACKﾈｼﾞ締め
    tole_pos = 5
    tole_ang = 2
   VALUE 404: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 504: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 105: ; GAS置き
    tole_pos = 5
    tole_ang = 2
   VALUE 205: ; GAS取り
    tole_pos = 10 ; (1022変更_置き位置ｽﾞﾚ大きくなる為)
    tole_ang = 2
   VALUE 305: ; GASﾈｼﾞ締め
    tole_pos = 5
    tole_ang = 2
   VALUE 405: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
   VALUE 505: ; 現状未使用
    tole_pos = 5
    tole_ang = 2
  END
;
  RETURN
;
.END

.PROGRAM md_send_vis_pos(.type,.id) #0; 計測座標をPLCへ送信
; /======================================================================/
; FUNCTION: 計測座標をPLCへ送信
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
;[.type]
; 1:ユニット組み込み位置のずれ
; 2:ロボット取り出し位置のずれ
; 3:ユニット把持位置のずれ
; 4:ロボット置き位置のずれ
; 11:検査１
; 12:検査２
; 13:検査３
; 14:検査４
; 15:検査５
; ※検査は未確定の為暫定
;
; 21:RBの歪み
; 22:吊り具の歪み
; 23:マテハンlongの歪み1
; 24:マテハンshortの歪み
; 25:ナットランナーM12の歪み
; 26:ナットランナーM20の歪み
; 27:マテハンlongの歪み2
; 28:マテハンlongの歪み3
; 29:マテハンlongの歪み4
; 30:マテハンshortの歪み2
; 31:マテハンshortの歪み3
; 32:マテハンshortの歪み4
;
; << ﾛｸﾞﾀｲﾌﾟの送信と照合 >>
  BITS ox_vis_log[1],7 = .type
  TIMER 1 = 0
  DO
    IF TIMER(1)>sys_timeout THEN
      CALL z.app_error("ﾋﾞｼﾞｮﾝﾛｸﾞ位置番号送信ｴﾗｰ")
    END
  UNTIL BITS(wx_vis_log_ans[1],7)==.type ; 照合OK  
;
  IF .type==1 THEN ; ﾕﾆｯﾄ組み込み位置のずれ
    IF cnt5[.id]==1 THEN
      cnt5[.id] = 101 ; -1用
    END
    .sendpos[1] = diff5_put_x[.id,cnt5[.id]-1]
    .sendpos[2] = diff5_put_y[.id,cnt5[.id]-1]
    .sendpos[3] = diff5_put_z[.id,cnt5[.id]-1]
    .sendpos[4] = diff5_put_rx[.id,cnt5[.id]-1]
    .sendpos[5] = diff5_put_ry[.id,cnt5[.id]-1]
    .sendpos[6] = diff5_put_rz[.id,cnt5[.id]-1]
  END
  IF .type==2 THEN ; ﾛﾎﾞｯﾄ取り出し位置のずれ
    IF cnt3[.id]==1 THEN
      cnt3[.id] = 101 ; -1用
    END
    .sendpos[1] = diff3_x[.id,cnt3[.id]-1]
    .sendpos[2] = diff3_y[.id,cnt3[.id]-1]
    .sendpos[3] = diff3_z[.id,cnt3[.id]-1]
    .sendpos[4] = diff3_rx[.id,cnt3[.id]-1]
    .sendpos[5] = diff3_ry[.id,cnt3[.id]-1]
    .sendpos[6] = diff3_rz[.id,cnt3[.id]-1]
  END
  IF .type==3 THEN ; ﾕﾆｯﾄ把持位置のずれ
    IF cnt5[.id]==1 THEN
      cnt5[.id] = 101 ; -1用
    END
    .sendpos[1] = diff5_have_x[.id,cnt5[.id]-1]
    .sendpos[2] = diff5_have_y[.id,cnt5[.id]-1]
    .sendpos[3] = diff5_have_z[.id,cnt5[.id]-1]
    .sendpos[4] = diff5_have_rx[.id,cnt5[.id]-1]
    .sendpos[5] = diff5_have_ry[.id,cnt5[.id]-1]
    .sendpos[6] = diff5_have_rz[.id,cnt5[.id]-1]
  END
  IF .type==4 THEN ; ﾛﾎﾞｯﾄ置き位置のずれ
    IF cnt5[.id]==1 THEN
      cnt5[.id] = 101 ; -1用
    END
    .sendpos[1] = diff5_x[.id,cnt5[.id]-1]
    .sendpos[2] = diff5_y[.id,cnt5[.id]-1]
    .sendpos[3] = diff5_z[.id,cnt5[.id]-1]
    .sendpos[4] = diff5_rx[.id,cnt5[.id]-1]
    .sendpos[5] = diff5_ry[.id,cnt5[.id]-1]
    .sendpos[6] = diff5_rz[.id,cnt5[.id]-1]
  END
  IF .type==21 THEN ; RBの歪み
    IF cnt1[.id]==1 THEN
      cnt1[.id] = 101 ; -1用
    END
    .sendpos[1] = diff1_x[.id,cnt1[.id]-1]
    .sendpos[2] = diff1_y[.id,cnt1[.id]-1]
    .sendpos[3] = diff1_z[.id,cnt1[.id]-1] ; 不要？
    .sendpos[4] = diff1_rx[.id,cnt1[.id]-1] ; 不要？
    .sendpos[5] = diff1_ry[.id,cnt1[.id]-1] ; 不要？
    .sendpos[6] = diff1_rz[.id,cnt1[.id]-1]
  END
  IF .type==22 THEN ; 吊り具の歪み
    IF cnt1[.id]==1 THEN
      cnt1[.id] = 101 ; -1用
    END
    .sendpos[1] = diff1_x[.id,cnt1[.id]-1]
    .sendpos[2] = diff1_y[.id,cnt1[.id]-1]
    .sendpos[3] = diff1_z[.id,cnt1[.id]-1] ; 不要？
    .sendpos[4] = diff1_rx[.id,cnt1[.id]-1] ; 不要？
    .sendpos[5] = diff1_ry[.id,cnt1[.id]-1] ; 不要？
    .sendpos[6] = diff1_rz[.id,cnt1[.id]-1]
  END
  IF .type==23 OR .type==27 OR .type==28 OR .type==29 THEN ; ﾏﾃﾊﾝlongの歪み1,2,3,4
    IF cnt1[.id]==1 THEN
      cnt1[.id] = 101 ; -1用
    END
    .sendpos[1] = diff1_x[.id,cnt1[.id]-1]
    .sendpos[2] = diff1_y[.id,cnt1[.id]-1]
    .sendpos[3] = diff1_z[.id,cnt1[.id]-1] ; 不要？
    .sendpos[4] = diff1_rx[.id,cnt1[.id]-1] ; 不要？
    .sendpos[5] = diff1_ry[.id,cnt1[.id]-1] ; 不要？
    .sendpos[6] = diff1_rz[.id,cnt1[.id]-1]
  END
  IF .type==24 OR .type==30 OR .type==31 OR .type==32 THEN ; ﾏﾃﾊﾝshortの歪み1,2,3,4
    IF cnt1[.id]==1 THEN
      cnt1[.id] = 101 ; -1用
    END
    .sendpos[1] = diff1_x[.id,cnt1[.id]-1]
    .sendpos[2] = diff1_y[.id,cnt1[.id]-1]
    .sendpos[3] = diff1_z[.id,cnt1[.id]-1] ; 不要？
    .sendpos[4] = diff1_rx[.id,cnt1[.id]-1] ; 不要？
    .sendpos[5] = diff1_ry[.id,cnt1[.id]-1] ; 不要？
    .sendpos[6] = diff1_rz[.id,cnt1[.id]-1]
  END
  IF .type==25 THEN ; ﾅｯﾄﾗﾝﾅｰM12の歪み
    IF cnt1[.id]==1 THEN
      cnt1[.id] = 101 ; -1用
    END
    .sendpos[1] = diff1_x[.id,cnt1[.id]-1]
    .sendpos[2] = diff1_y[.id,cnt1[.id]-1]
    .sendpos[3] = diff1_z[.id,cnt1[.id]-1] ; 不要？
    .sendpos[4] = diff1_rx[.id,cnt1[.id]-1] ; 不要？
    .sendpos[5] = diff1_ry[.id,cnt1[.id]-1] ; 不要？
    .sendpos[6] = diff1_rz[.id,cnt1[.id]-1]
  END
  IF .type==26 THEN ; ﾅｯﾄﾗﾝﾅｰM20の歪み
    IF cnt1[.id]==1 THEN
      cnt1[.id] = 101 ; -1用
    END
    .sendpos[1] = diff1_x[.id,cnt1[.id]-1]
    .sendpos[2] = diff1_y[.id,cnt1[.id]-1]
    .sendpos[3] = diff1_z[.id,cnt1[.id]-1] ; 不要？
    .sendpos[4] = diff1_rx[.id,cnt1[.id]-1] ; 不要？
    .sendpos[5] = diff1_ry[.id,cnt1[.id]-1] ; 不要？
    .sendpos[6] = diff1_rz[.id,cnt1[.id]-1]
  END
;  
  IF .sendpos[1]<0 THEN ; Xがﾏｲﾅｽ
    SIGNAL ox_vis_x_pos_m ; ﾏｲﾅｽbitをON
  ELSE
    SIGNAL -ox_vis_x_pos_m ; ﾏｲﾅｽbitをOFF
  END
  IF .sendpos[2]<0 THEN ; Yがﾏｲﾅｽ
    SIGNAL ox_vis_y_pos_m ; ﾏｲﾅｽbitをON
  ELSE
    SIGNAL -ox_vis_y_pos_m ; ﾏｲﾅｽbitをOFF
  END
  IF .sendpos[3]<0 THEN ; Zがﾏｲﾅｽ
    SIGNAL ox_vis_z_pos_m ; ﾏｲﾅｽbitをON
  ELSE
    SIGNAL -ox_vis_z_pos_m ; ﾏｲﾅｽbitをOFF
  END
  IF .sendpos[4]<0 THEN ; Oがﾏｲﾅｽ
    SIGNAL ox_vis_o_pos_m ; ﾏｲﾅｽbitをON
  ELSE
    SIGNAL -ox_vis_o_pos_m ; ﾏｲﾅｽbitをOFF
  END
  IF .sendpos[5]<0 THEN ; Aがﾏｲﾅｽ
    SIGNAL ox_vis_a_pos_m ; ﾏｲﾅｽbitをON
  ELSE
    SIGNAL -ox_vis_a_pos_m ; ﾏｲﾅｽbitをOFF
  END
  IF .sendpos[6]<0 THEN ; Tがﾏｲﾅｽ
    SIGNAL ox_vis_t_pos_m ; ﾏｲﾅｽbitをON
  ELSE
    SIGNAL -ox_vis_t_pos_m ; ﾏｲﾅｽbitをOFF
  END
;
  FOR .i = 1 TO 6
    .$p[1] = $ENCODE(/L,.sendpos[.i]) ; 文字に変換
    .$val = $DECODE(.$p[1],".",0) ; 小数点前を取り出し
    .$c = $LEFT(.$val,1)
    IF .$c=="-" THEN ; -符号なら
      .$temp = $DECODE(.$val,"-",1) ; -を削除
    END
    $log_val[.i] = .$val ; log
    .mae[.i] = VAL(.$val) ; 数値に変換
;
    $log_p[.i] = .$p[1] ; log
    IF .$p[1]=="" THEN ; 数字が0.000だと.以降が消失するため処理ｽｷｯﾌﾟする
      .ushiro[.i] = 0
      GOTO skip
    END
    .$temp = $DECODE(.$p[1],".",1) ; .を削除
    .$val = $DECODE(.$p[1],".",0) ; 小数点後を取り出し
    .$val = $LEFT(.$val,3) ; 3桁目までにする
    $log_val2[.i] = .$val ; log	
    .ushiro[.i] = VAL(.$val) ; 数値に変換
skip:
    log_mae[.i] = .mae[.i] ; log
    log_ushiro[.i] = .ushiro[.i] ; log
  END
;
;<<PLCに座標送信>>
  BITS ox_vis_x_pos1[1],14 = .mae[1]
  BITS ox_vis_x_pos2[1],14 = .ushiro[1]
  BITS ox_vis_y_pos1[1],14 = .mae[2]
  BITS ox_vis_y_pos2[1],14 = .ushiro[2]
  BITS ox_vis_z_pos1[1],14 = .mae[3]
  BITS ox_vis_z_pos2[1],14 = .ushiro[3]
  BITS ox_vis_o_pos1[1],14 = .mae[4]
  BITS ox_vis_o_pos2[1],14 = .ushiro[4]
  BITS ox_vis_a_pos1[1],14 = .mae[5]
  BITS ox_vis_a_pos2[1],14 = .ushiro[5]
  BITS ox_vis_t_pos1[1],14 = .mae[6]
  BITS ox_vis_t_pos2[1],14 = .ushiro[6]
;
  SIGNAL ox_trg_vis ; ﾋﾞｼﾞｮﾝ座標PLC出力ﾄﾘｶﾞ
  TIMER 1 = 0
  DO
    IF TIMER(1)>sys_timeout THEN
      SIGNAL -ox_trg_vis
      CALL z.app_error("ﾋﾞｼﾞｮﾝﾛｸﾞ送信ｴﾗｰ")
    END
  UNTIL SIG(wx_trg_vis) ; ﾋﾞｼﾞｮﾝ座標PLC受信完了
  SIGNAL -ox_trg_vis
;
  RETURN
;
.END

.PROGRAM bolt_vision_exe(.mes_no,.bolt_size) ; ボルトビジョン計測
;撮影位置　bolt_grab
;ボルトビジョン計測計測
;.mes_no = 47       ;仮
;.bolt_size = 12    ;仮
  bolt_pos_num12 = 0
  bolt_pos_num20 = 0
;Set ip address of IPC
  CALL mm_init_skt(mm_ip[1],mm_ip[2],mm_ip[3],mm_ip[4],mm_port[1])
  TWAIT 0.1
  CALL mm_start_vis(.mes_no,0,2,#start_vis)
  TWAIT 1
  CASE .bolt_size OF
   VALUE 12:
    CALL mm_get_visdata(.mes_no,bolt_pos_num12,ret2)
    IF ret2<>1100 THEN
      v.err = 4
      CALL vget_error_mess(v.err,$v.err_message); error message
      RETURN
    ELSE
      v.err = 0
    END
    FOR bolt_num = 1 TO bolt_pos_num12
      CALL mm_get_pose(bolt_num,&bolt12[bolt_num],label[bolt_num],speed[bolt_num])
    END
   VALUE 20:
    CALL mm_get_visdata(.mes_no,bolt_pos_num20,ret2)
    IF ret2<>1100 THEN
      v.err = 4
      CALL vget_error_mess(v.err,$v.err_message); error message
      RETURN
    ELSE
      v.err = 0
    END
    FOR bolt_num = 1 TO bolt_pos_num20
      CALL mm_get_pose(bolt_num,&bolt20[bolt_num],label[bolt_num],speed[bolt_num])
    END
  END
.END

.PROGRAM bolt_flg(.bolt_size)
;ボルト置き場にボルトがある場合フラグがON
;
;.bolt_size = 12   ;仮
  CASE .bolt_size OF
   VALUE 12:         ;bolt size 12
    .bolt_max = 24
    FOR .i = 1 TO .bolt_max
      bolt_flg12[.i] = 0
    END
    FOR .m = 1 TO .bolt_max
      FOR .n = 1 TO bolt_pos_num12
        .bolt_distance[.m,.n] = DISTANCE(master_bolt12[.m],bolt12[.n])
        IF (0<.bolt_distance[.m,.n]) AND (.bolt_distance[.m,.n]<2) THEN
          bolt_flg12[.m] = ON
        END
      END
    END
;
   VALUE 20:        ;bolt size 20
    .bolt_max = 8
    FOR .i = 1 TO .bolt_max
      bolt_flg20[.i] = 0
    END
    FOR .m = 1 TO .bolt_max
      FOR .n = 1 TO bolt_pos_num20
        .bolt_distance[.m,.n] = DISTANCE(master_bolt20[.m],bolt20[.n])
        IF (0<.bolt_distance[.m,.n]) AND (.bolt_distance[.m,.n]<2) THEN
          bolt_flg20[.m] = ON
        END
      END
    END
  END
.END


;***********************************************************
;動作パラメータ設定関連
;***********************************************************

.PROGRAM z.aux(.sp,.acu,.acc,.dec,.$abs); 補助設定
; /======================================================================/
; FUNCTION: 補助設定
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  IF .$abs=="mms" THEN
    SPEED .sp MM/S
  ELSE
    SPEED .sp
  END
  ACCURACY .acu
  ACCEL .acc
  DECEL .dec
;
  RETURN
;
.END

.PROGRAM md_recv_gain(.pos) #86; 各軸ﾁｪﾝｼﾞｹﾞｲﾝ値受信
; /======================================================================/
; FUNCTION: 各軸ﾁｪﾝｼﾞｹﾞｲﾝ値受信
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
;[.pos]
;1:ﾏﾃﾊﾝ取り時
;2:ﾕﾆｯﾄ取り/ﾏﾃﾊﾝ置き時
;3:ﾕﾆｯﾄ置き時
;11:ﾎﾞﾙﾄ吸着時
;12:ﾎﾞﾙﾄ水平締め時
;13:ﾎﾞﾙﾄ垂直締め時
;
  BITS ox_gain_pos[1],7 = .pos
  TIMER 1 = 0
  DO
    IF TIMER(1)>sys_timeout THEN
      CALL z.app_error("ﾁｪﾝｼﾞｹﾞｲﾝ番号受信ｴﾗｰ")
    END
  UNTIL BITS(wx_gain_pos_ans[1],7)==.pos ; 照合OK  
;
;
  TIMER 1 = 0
  DO
    .gain[1] = BITS(wx_jt1gain[1],7) ; JT1ｹﾞｲﾝ値受信
    .gain[2] = BITS(wx_jt2gain[1],7) ; JT2ｹﾞｲﾝ値受信
    .gain[3] = BITS(wx_jt3gain[1],7) ; JT3ｹﾞｲﾝ値受信
    .gain[4] = BITS(wx_jt4gain[1],7) ; JT4ｹﾞｲﾝ値受信
    .gain[5] = BITS(wx_jt5gain[1],7) ; JT5ｹﾞｲﾝ値受信
    .gain[6] = BITS(wx_jt6gain[1],7) ; JT6ｹﾞｲﾝ値受信
    TWAIT ascycle
    BITS ox_jt1gain_ans[1],7 = .gain[1]
    BITS ox_jt2gain_ans[1],7 = .gain[2]
    BITS ox_jt3gain_ans[1],7 = .gain[3]
    BITS ox_jt4gain_ans[1],7 = .gain[4]
    BITS ox_jt5gain_ans[1],7 = .gain[5]
    BITS ox_jt6gain_ans[1],7 = .gain[6]
;
    IF TIMER(1)>sys_timeout THEN
      CALL z.app_error("ﾁｪﾝｼﾞｹﾞｲﾝ値受信ｴﾗｰ")
    END
  UNTIL SIG(wx_gain_ok) ; 照合OK
;
;ｹﾞｲﾝ設定(1~6軸,%)
  CGNGAIN .gain[1],.gain[2],.gain[3],.gain[4],.gain[5],.gain[6]
;
  BITS ox_gain_pos[1],7 = 0 ; ﾘｾｯﾄ
;
  RETURN
;
.END



;***********************************************************
;マテハン・クレーン・ナットランナ動作関連
;***********************************************************


.PROGRAM md_atc_ctrl(.$param1,.$param2) #80; ATC動作
; /======================================================================/
; FUNCTION: ATC動作
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  UTIMER @timeout = 0
;
  IF (.$param1=="吊り具上" OR .$param1=="吊り具横") AND .$param2=="ﾁｬｯｸ" THEN
    IF .$param1=="吊り具上" THEN
      .no = 1
    END
    IF .$param1=="吊り具横" THEN
      .no = 2
    END
    SIGNAL ox_arrive_atc[.no] ; 取置位置到着
    SIGNAL ox_close_atc[.no],-ox_open_atc[.no] ; ATCﾁｬｯｸ
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error(.$param1+"ﾁｬｯｸｴﾗｰ")
      END
    UNTIL SIG(wx_close_atc[.no]) AND SIG(wx_comp_cl_atc[.no]) ; ﾁｬｯｸ中/ﾁｬｯｸ完了
    SIGNAL -ox_arrive_atc[.no]
    SIGNAL -ox_close_atc[.no],-ox_open_atc[.no]
  END
;
;
  IF (.$param1=="吊り具上" OR .$param1=="吊り具横") AND .$param2=="ｱﾝﾁｬｯｸ" THEN
    IF .$param1=="吊り具上" THEN
      .no = 1
    END
    IF .$param1=="吊り具横" THEN
      .no = 2
    END
    SIGNAL ox_arrive_atc[.no] ; 取置位置到着
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error(.$param1+"ｱﾝﾁｬｯｸ不可ｴﾗｰ")
      END
    UNTIL SIG(wx_atc_open_ok[.no]) ; ｱﾝﾁｬｯｸ可
    SIGNAL -ox_close_atc[.no],ox_open_atc[.no] ; ATCｱﾝﾁｬｯｸ
    UTIMER @timeout = 0
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error(.$param1+"ｱﾝﾁｬｯｸｴﾗｰ")
      END
    UNTIL SIG(wx_comp_op_atc[.no]) ; ｱﾝﾁｬｯｸ完了
    SIGNAL -ox_arrive_atc[.no]
    SIGNAL -ox_close_atc[.no],-ox_open_atc[.no]
    TWAIT 1 ; 追加待ち TIMER    
  END
;
;
  IF (.$param1=="ﾏﾃﾊﾝlong" OR .$param1=="ﾏﾃﾊﾝshort") AND .$param2=="ﾁｬｯｸ" THEN
    IF .$param1=="ﾏﾃﾊﾝlong" THEN
      .no = 3
    END
    IF .$param1=="ﾏﾃﾊﾝshort" THEN
      .no = 4
    END
;
    SIGNAL ox_arrive_atc[.no] ; 取置位置到着
    SIGNAL ox_close_atc[.no],-ox_open_atc[.no] ; ATCﾁｬｯｸ
    DO
      IF UTIMER(@timeout)>180 THEN ; IAI起動時間があるので長め
        CALL z.app_error(.$param1+"ﾁｬｯｸｴﾗｰ")
      END
;UNTIL SIG(wx_close_atc[.no]) AND SIG(wx_comp_cl_atc[.no]) AND SIG(wx_iai_home) ; ﾁｬｯｸ中/ﾁｬｯｸ完了/IAI原点
    UNTIL SIG(wx_close_atc[.no]) AND SIG(wx_comp_cl_atc[.no]) ; ﾁｬｯｸ中/ﾁｬｯｸ完了(ﾀｸﾄｱｯﾌﾟ用に変更 0626)
    SIGNAL -ox_arrive_atc[.no]
    SIGNAL -ox_close_atc[.no],-ox_open_atc[.no]
  END
;
;
  IF (.$param1=="ﾏﾃﾊﾝlong" OR .$param1=="ﾏﾃﾊﾝshort") AND .$param2=="ｱﾝﾁｬｯｸ" THEN
    IF .$param1=="ﾏﾃﾊﾝlong" THEN
      .no = 3
    END
    IF .$param1=="ﾏﾃﾊﾝshort" THEN
      .no = 4
    END
    SIGNAL ox_arrive_atc[.no] ; 取置位置到着  
    UTIMER @timeout = 0
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error(.$param1+"ｱﾝﾁｬｯｸ不可ｴﾗｰ")
      END
    UNTIL SIG(wx_atc_open_ok[.no]) ; ｱﾝﾁｬｯｸ可
;
    SIGNAL -ox_close_atc[.no],ox_open_atc[.no] ; ATCｱﾝﾁｬｯｸ
    UTIMER @timeout = 0
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error(.$param1+"ｱﾝﾁｬｯｸｴﾗｰ")
      END
    UNTIL SIG(wx_comp_op_atc[.no]) ; ｱﾝﾁｬｯｸ完了
    SIGNAL -ox_arrive_atc[.no]
    SIGNAL -ox_close_atc[.no],-ox_open_atc[.no]
    TWAIT 1 ; 追加待ち TIMER
  END
;
;
  IF (.$param1=="ﾅｯﾄﾗﾝﾅｰM12_1" OR .$param1=="ﾅｯﾄﾗﾝﾅｰM12_2" OR .$param1=="ﾅｯﾄﾗﾝﾅｰM20") AND .$param2=="ﾁｬｯｸ" THEN
    IF .$param1=="ﾅｯﾄﾗﾝﾅｰM12_1" THEN
      .no = 5
    END
    IF .$param1=="ﾅｯﾄﾗﾝﾅｰM12_2" THEN
      .no = 6
    END
    IF .$param1=="ﾅｯﾄﾗﾝﾅｰM20" THEN
      .no = 7
    END
    SIGNAL ox_arrive_atc[.no] ; 取置位置到着
;.no = 5 ; ここから共通で5
    SIGNAL ox_close_atc[5],-ox_open_atc[5] ; ATCﾁｬｯｸ
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error(.$param1+"ﾁｬｯｸｴﾗｰ")
      END
    UNTIL SIG(wx_close_atc[.no]) AND SIG(wx_comp_cl_atc[.no]) ; ﾁｬｯｸ中/ﾁｬｯｸ完了
    SIGNAL -ox_arrive_atc[.no]
    SIGNAL -ox_close_atc[5],-ox_open_atc[5]
  END
;
;
  IF (.$param1=="ﾅｯﾄﾗﾝﾅｰM12_1" OR .$param1=="ﾅｯﾄﾗﾝﾅｰM12_2" OR .$param1=="ﾅｯﾄﾗﾝﾅｰM20") AND .$param2=="ｱﾝﾁｬｯｸ" THEN
    IF .$param1=="ﾅｯﾄﾗﾝﾅｰM12_1" THEN
      .no = 5
    END
    IF .$param1=="ﾅｯﾄﾗﾝﾅｰM12_2" THEN
      .no = 6
    END
    IF .$param1=="ﾅｯﾄﾗﾝﾅｰM20" THEN
      .no = 7
    END
    SIGNAL ox_arrive_atc[.no] ; 取置位置到着
;.no = 5 ; ここから共通で5
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error(.$param1+"ｱﾝﾁｬｯｸ不可ｴﾗｰ")
      END
    UNTIL SIG(wx_atc_open_ok[.no]) ; ｱﾝﾁｬｯｸ可
    SIGNAL -ox_close_atc[5],ox_open_atc[5] ; ATCｱﾝﾁｬｯｸ
    UTIMER @timeout = 0
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error(.$param1+"ｱﾝﾁｬｯｸｴﾗｰ")
      END
    UNTIL SIG(wx_comp_op_atc[.no]) ; ｱﾝﾁｬｯｸ完了
    SIGNAL -ox_arrive_atc[.no]
    SIGNAL -ox_close_atc[5],-ox_open_atc[5]
    TWAIT 1 ; 追加待ち TIMER    
  END
;
  RETURN
;
.END

.PROGRAM md_crane_set(.$param) #22; ｱｼｽﾄｸﾚｰﾝ設定変更
; /======================================================================/
; FUNCTION: ｱｼｽﾄｸﾚｰﾝ設定変更
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  IF .$param=="空荷" OR .$param=="吊り具" THEN ; (0731変更)
    SIGNAL ox_crn_pos[1],-ox_crn_pos[2],-ox_crn_pos[3] ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ:空荷
    GOTO next
  END
  IF .$param=="ﾏﾃﾊﾝ把持" THEN
    SIGNAL -ox_crn_pos[1],ox_crn_pos[2],-ox_crn_pos[3] ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ:ﾏﾃﾊﾝ把持
    GOTO next
  END
  IF .$param=="ﾕﾆｯﾄ把持" THEN
    SIGNAL -ox_crn_pos[1],-ox_crn_pos[2],ox_crn_pos[3] ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ:ﾕﾆｯﾄ把持
    GOTO next
  END
  HALT ; 異常
;
;
next:
  TWAIT 0.5 ; PLCのﾊﾟﾗﾒｰﾀ選択信号受信待ちﾀｲﾏ
  PULSE ox_crn_paramset,pulse_tim ; ｱｼｽﾄｸﾚｰﾝﾊﾟﾗﾒｰﾀ変更指令
;
  UTIMER @timeout = 0
  DO
    IF UTIMER(@timeout)>sys_timeout THEN
      CALL z.app_error("ｱｼｽﾄｸﾚｰﾝ_"+.$param+"_ﾊﾟﾗﾒｰﾀ変更ｴﾗｰ")
    END
  UNTIL SIG(wx_crn_paramset) ; ﾊﾟﾗﾒｰﾀ変更完了
  SIGNAL -ox_crn_pos[1],-ox_crn_pos[2],-ox_crn_pos[3]
  SIGNAL -ox_crn_paramset
;
  RETURN
;
.END

.PROGRAM md_crane_move(.$param) #22; ｱｼｽﾄｸﾚｰﾝ動作変更
; /======================================================================/
; FUNCTION: ｱｼｽﾄｸﾚｰﾝ動作変更
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  UTIMER @timeout = 0
;
  IF .$param=="待機" THEN
    PULSE ox_crn_freemode,pulse_tim ; 待機状態に変更
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error("ｱｼｽﾄｸﾚｰﾝ_"+.$param+"_状態変更ｴﾗｰ")
      END
    UNTIL SIG(wx_crn_freemode) AND SIG(wx_crn_pos[1]) ; 待機変更完了/待機中
    SIGNAL -ox_crn_freemode
  END
;
;
  IF .$param=="ｱｼｽﾄUP" THEN
;
    PULSE ox_crn_as_up,pulse_tim ; ｱｼｽﾄｱｯﾌﾟに変更
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error("ｱｼｽﾄｸﾚｰﾝ_"+.$param+"_状態変更ｴﾗｰ")
      END
    UNTIL SIG(wx_crn_as_up) AND SIG(wx_crn_pos[2]) ; ｱｼｽﾄｱｯﾌﾟ変更完了/ﾊﾞﾗﾝｽ中
    SIGNAL -ox_crn_as_up
  END
;
;
  IF .$param=="ｱｼｽﾄDOWN" THEN
    PULSE ox_crn_as_down,pulse_tim ; ｱｼｽﾄｸﾚｰﾝｱｼｽﾄﾀﾞｳﾝ指令
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error("ｱｼｽﾄｸﾚｰﾝ_"+.$param+"_状態変更ｴﾗｰ")
      END
    UNTIL SIG(wx_crn_as_down) AND SIG(wx_crn_pos[1]) ; ｱｼｽﾄﾀﾞｳﾝ指令完了/待機中
    SIGNAL -ox_crn_as_down
  END
;
;
  IF .$param=="ｱｼｽﾄ減少" THEN
    PULSE ox_crn_as_dec,pulse_tim ; ｱｼｽﾄ減少に変更
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error("ｱｼｽﾄｸﾚｰﾝ_"+.$param+"_状態変更ｴﾗｰ")
      END
    UNTIL SIG(wx_crn_as_dec) AND (SIG(wx_crn_pos[2]) OR SIG(wx_crn_pos[1])) ; ｱｼｽﾄｱｯﾌﾟ変更完了/ﾊﾞﾗﾝｽ中or待機中(0801追加)
    SIGNAL -ox_crn_as_dec
  END
;
;
  TWAIT 1
;
  RETURN
;
.END

.PROGRAM md_ctrl_iai(.$ope) #12; IAI(ﾏﾃﾊﾝ爪)操作
; /======================================================================/
; FUNCTION: IAI(ﾏﾃﾊﾝ爪)操作
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  TIMER 1 = 0
;
  IF .$ope=="home" THEN
    SIGNAL ox_iai_home,-ox_iai_open,-ox_iai_close ; 原点に移動
    DO
      IF TIMER(1)>sys_timeout THEN
        CALL z.app_error("IAI原点移動ｴﾗｰ")
      END
    UNTIL SIG(wx_iai_home) ; 原点位置
  END
;
  IF .$ope=="open" THEN
    SIGNAL -ox_iai_home,ox_iai_open,-ox_iai_close ; 開き移動
    DO
      IF TIMER(1)>sys_timeout THEN
        CALL z.app_error("IAI開き移動ｴﾗｰ")
      END
    UNTIL SIG(wx_iai_open) ; 開き位置
  END
;
  IF .$ope=="close" THEN
    SIGNAL -ox_iai_home,-ox_iai_open,ox_iai_close ; 閉じ移動
    DO
      IF TIMER(1)>sys_timeout THEN
        CALL z.app_error("IAI閉じ移動ｴﾗｰ")
      END
    UNTIL SIG(wx_iai_close) ; 閉じ位置
  END
;
;
  SIGNAL -ox_iai_home,-ox_iai_open,-ox_iai_close ; 操作終了後はOFF
;
  RETURN
;
.END

.PROGRAM md_recv_natrpm(.no,.rpm) ; ﾅｯﾄﾗﾝﾅｰ回転数受信
; /======================================================================/
; FUNCTION: ﾅｯﾄﾗﾝﾅｰ回転数受信
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
;[.no]
;1:空転
;2:ﾄﾙｸ締め
;3:緩め
;.rpm→返値
;
  SIGNAL ox_rot_pos[.no]
  TIMER 1 = 0
  DO
    IF TIMER(1)>sys_timeout THEN
      CALL z.app_error("ﾅｯﾄﾗﾝﾅｰ回転数番号受信ｴﾗｰ")
    END
  UNTIL SIG(wx_rot_pos_ans[.no]) ; 照合OK    
;
  TIMER 1 = 0
  DO
    .rpm = BITS(wx_nat_rpm[1],12) ; ﾅｯﾄﾗﾝﾅｰ回転数受信
    TWAIT ascycle
    BITS ox_nat_rpm_ans[1],12 = .rpm
;
    IF TIMER(1)>sys_timeout THEN
      CALL z.app_error("ﾅｯﾄﾗﾝﾅｰ回転数受信ｴﾗｰ")
    END
  UNTIL SIG(wx_nat_rpm_ok) ; 照合OK
;
;ﾘｾｯﾄ
  SIGNAL -ox_rot_pos[.no]
  BITS ox_nat_rpm_ans[1],12 = 0
;
  RETURN
;
.END

;***********************************************************
;エラー処理関連
;***********************************************************
.PROGRAM z.app_error(.$err) #0; ｱﾌﾟﾘｴﾗｰ処理
; /======================================================================/
; FUNCTION: ｱﾌﾟﾘｴﾗｰ処理
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  err_num = 0 ; 初期化
;  
;<< ｼｽﾃﾑｴﾗｰ >>
  IF .$err=="autostart.pc停止ｴﾗｰ" THEN
    err_num = 1001
  END
  IF .$err=="品種選択ｴﾗｰ" THEN
    err_num = 1002
  END
  IF .$err=="手動退避後の退避点範囲外ｴﾗｰ" THEN
    err_num = 1003
  END
  IF .$err=="ﾕﾆｯﾄ置き台ﾕﾆｯﾄ選択無しｴﾗｰ" THEN ; (0801追加)
    err_num = 1004
  END
  IF .$err=="ｲﾝﾀｰﾛｯｸﾀｲﾑｱｳﾄｴﾗｰ" THEN ; (1020追加)
    err_num = 1005
  END
;
;<< 把持系ｴﾗｰ >> 
  IF .$err=="吊り具上ﾁｬｯｸｴﾗｰ" THEN
    err_num = 2001
  END
  IF .$err=="吊り具横ﾁｬｯｸｴﾗｰ" THEN
    err_num = 2002
  END
  IF .$err=="吊り具上ｱﾝﾁｬｯｸ不可ｴﾗｰ" THEN
    err_num = 2003
  END
  IF .$err=="吊り具横ｱﾝﾁｬｯｸ不可ｴﾗｰ" THEN
    err_num = 2004
  END
  IF .$err=="吊り具上ｱﾝﾁｬｯｸｴﾗｰ" THEN
    err_num = 2005
  END
  IF .$err=="吊り具横ｱﾝﾁｬｯｸｴﾗｰ" THEN
    err_num = 2006
  END
  IF .$err=="ﾏﾃﾊﾝlongﾁｬｯｸｴﾗｰ" THEN
    err_num = 2007
  END
  IF .$err=="ﾏﾃﾊﾝshortﾁｬｯｸｴﾗｰ" THEN
    err_num = 2008
  END
  IF .$err=="ﾏﾃﾊﾝlongｱﾝﾁｬｯｸ不可ｴﾗｰ" THEN
    err_num = 2009
  END
  IF .$err=="ﾏﾃﾊﾝshortｱﾝﾁｬｯｸ不可ｴﾗｰ" THEN
    err_num = 2010
  END
  IF .$err=="ﾏﾃﾊﾝlongｱﾝﾁｬｯｸｴﾗｰ" THEN
    err_num = 2011
  END
  IF .$err=="ﾏﾃﾊﾝshortｱﾝﾁｬｯｸｴﾗｰ" THEN
    err_num = 2012
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1ﾁｬｯｸｴﾗｰ" THEN
    err_num = 2013
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2ﾁｬｯｸｴﾗｰ" THEN
    err_num = 2014
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20ﾁｬｯｸｴﾗｰ" THEN
    err_num = 2015
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1ｱﾝﾁｬｯｸ不可ｴﾗｰ" THEN
    err_num = 2016
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2ｱﾝﾁｬｯｸ不可ｴﾗｰ" THEN
    err_num = 2017
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20ｱﾝﾁｬｯｸ不可ｴﾗｰ" THEN
    err_num = 2018
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1ｱﾝﾁｬｯｸｴﾗｰ" THEN
    err_num = 2019
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2ｱﾝﾁｬｯｸｴﾗｰ" THEN
    err_num = 2020
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20ｱﾝﾁｬｯｸｴﾗｰ" THEN
    err_num = 2021
  END
  IF .$err=="ﾏﾃﾊﾝlong_爪ｾﾝｻのﾕﾆｯﾄ検知ｴﾗｰ" THEN
    err_num = 2022
  END
  IF .$err=="ﾏﾃﾊﾝshort_爪ｾﾝｻのﾕﾆｯﾄ検知ｴﾗｰ" THEN
    err_num = 2023
  END
;
;<< ｱｼｽﾄｸﾚｰﾝｴﾗｰ >>  
  IF .$err=="ｱｼｽﾄｸﾚｰﾝ_待機_状態変更ｴﾗｰ" THEN
    err_num = 3001
  END
  IF .$err=="ｱｼｽﾄｸﾚｰﾝ_ｱｼｽﾄUP_状態変更ｴﾗｰ" THEN
    err_num = 3002
  END
  IF .$err=="ｱｼｽﾄｸﾚｰﾝ_ｱｼｽﾄDOWN_状態変更ｴﾗｰ" THEN
    err_num = 3003
  END
  IF .$err=="ｱｼｽﾄｸﾚｰﾝ_ｱｼｽﾄ減少_状態変更ｴﾗｰ" THEN
    err_num = 3004
  END
  IF .$err=="ｱｼｽﾄｸﾚｰﾝ_空荷_ﾊﾟﾗﾒｰﾀ変更ｴﾗｰ" THEN
    err_num = 3005
  END
  IF .$err=="ｱｼｽﾄｸﾚｰﾝ_ﾏﾃﾊﾝ把持_ﾊﾟﾗﾒｰﾀ変更ｴﾗｰ" THEN
    err_num = 3006
  END
  IF .$err=="ｱｼｽﾄｸﾚｰﾝ_ﾕﾆｯﾄ把持_ﾊﾟﾗﾒｰﾀ変更ｴﾗｰ" THEN
    err_num = 3007
  END
  IF .$err=="ｱｼｽﾄｸﾚｰﾝ_ｻｲｸﾙ運転中HOLDｴﾗｰ" THEN ; autostartで発報(0801追加)
    err_num = 3008
  END
;
;<< IAIｴﾗｰ >>
  IF .$err=="IAI原点移動ｴﾗｰ" THEN
    err_num = 4001
  END
  IF .$err=="IAI開き移動ｴﾗｰ" THEN
    err_num = 4002
  END
  IF .$err=="IAI閉じ移動ｴﾗｰ" THEN
    err_num = 4003
  END
;
;<< ﾊﾟﾗﾒｰﾀ送受信ｴﾗｰ >> 
  IF .$err=="ﾁｪﾝｼﾞｹﾞｲﾝ番号受信ｴﾗｰ" THEN
    err_num = 5001
  END
  IF .$err=="ﾁｪﾝｼﾞｹﾞｲﾝ値受信ｴﾗｰ" THEN
    err_num = 5002
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰ回転数番号受信ｴﾗｰ" THEN
    err_num = 5003
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰ回転数受信ｴﾗｰ" THEN
    err_num = 5004
  END
  IF .$err=="ﾋﾞｼﾞｮﾝ補正ID位置番号受信ｴﾗｰ" THEN
    err_num = 5005
  END
  IF .$err=="ﾋﾞｼﾞｮﾝ補正ID受信ｴﾗｰ" THEN
    err_num = 5006
  END
  IF .$err=="ﾋﾞｼﾞｮﾝﾛｸﾞ位置番号送信ｴﾗｰ" THEN
    err_num = 5007
  END
  IF .$err=="ﾋﾞｼﾞｮﾝﾛｸﾞ送信ｴﾗｰ" THEN
    err_num = 5008
  END
;
;<< 在籍ｴﾗｰ >>
  IF .$err=="ﾏﾃﾊﾝlong置き台無在籍ｴﾗｰ" THEN
    err_num = 6001
  END
  IF .$err=="ﾏﾃﾊﾝshort置き台無在籍ｴﾗｰ" THEN
    err_num = 6002
  END
  IF .$err=="ﾏﾃﾊﾝlong置き台在籍有ｴﾗｰ" THEN
    err_num = 6003
  END
  IF .$err=="ﾏﾃﾊﾝshort置き台在籍有ｴﾗｰ" THEN
    err_num = 6004
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1置き台在籍無ｴﾗｰ" THEN
    err_num = 6005
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2置き台在籍無ｴﾗｰ" THEN
    err_num = 6006
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20置き台在籍無ｴﾗｰ" THEN
    err_num = 6007
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1置き台在籍有ｴﾗｰ" THEN
    err_num = 6008
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2置き台在籍有ｴﾗｰ" THEN
    err_num = 6009
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20置き台在籍有ｴﾗｰ" THEN
    err_num = 6010
  END
  IF .$err=="吊り具置き台在籍無ｴﾗｰ" THEN
    err_num = 6011
  END
  IF .$err=="吊り具置き台在籍有ｴﾗｰ" THEN
    err_num = 6012
  END
;
;<< ﾋﾞｼﾞｮﾝｴﾗｰ >>
  IF .$err=="RF_ﾌﾚｰﾑ1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7001
  END
  IF .$err=="RF_ﾌﾚｰﾑ2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7002
  END
  IF .$err=="RF_ﾌﾚｰﾑ3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7003
  END
  IF .$err=="EXHAUST_ﾌﾚｰﾑ1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7004
  END
  IF .$err=="EXHAUST_ﾌﾚｰﾑ2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7005
  END
  IF .$err=="EXHAUST_ﾌﾚｰﾑ3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7006
  END
  IF .$err=="REACTOR_ﾌﾚｰﾑ1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7007
  END
  IF .$err=="REACTOR_ﾌﾚｰﾑ2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7008
  END
  IF .$err=="REACTOR_ﾌﾚｰﾑ3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7009
  END
  IF .$err=="TOPRACK_ﾌﾚｰﾑ1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7010
  END
  IF .$err=="TOPRACK_ﾌﾚｰﾑ2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7011
  END
  IF .$err=="TOPRACK_ﾌﾚｰﾑ3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7012
  END
  IF .$err=="GAS_ﾌﾚｰﾑ1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7013
  END
  IF .$err=="GAS_ﾌﾚｰﾑ2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7014
  END
  IF .$err=="GAS_ﾌﾚｰﾑ3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7015
  END
;
  IF .$err=="RF_ﾕﾆｯﾄ底面1点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7101
  END
  IF .$err=="RF_ﾕﾆｯﾄ底面2点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7102
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ底面1点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7103
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ底面2点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7104
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ底面1点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7105
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ底面2点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7106
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ底面1点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7107
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ底面2点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7108
  END
  IF .$err=="GAS_ﾕﾆｯﾄ底面1点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7109
  END
  IF .$err=="GAS_ﾕﾆｯﾄ底面2点目_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7110
  END
;
  IF .$err=="RF_ﾕﾆｯﾄ上面1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7201
  END
  IF .$err=="RF_ﾕﾆｯﾄ上面2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7202
  END
  IF .$err=="RF_ﾕﾆｯﾄ上面3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7203
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ上面1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7204
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ上面2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7205
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ上面3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7206
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ上面1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7207
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ上面2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7208
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ上面3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7209
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ上面1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7210
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ上面2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7211
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ上面3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7212
  END
  IF .$err=="GAS_ﾕﾆｯﾄ上面1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7213
  END
  IF .$err=="GAS_ﾕﾆｯﾄ上面2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7214
  END
  IF .$err=="GAS_ﾕﾆｯﾄ上面3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7215
  END
;
  IF .$err=="RF_ねじ締め計測1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7301
  END
  IF .$err=="RF_ねじ締め計測2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7302
  END
  IF .$err=="RF_ねじ締め計測3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7303
  END
  IF .$err=="EXHAUST_ねじ締め計測1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7304
  END
  IF .$err=="EXHAUST_ねじ締め計測2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7305
  END
  IF .$err=="EXHAUST_ねじ締め計測3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7306
  END
  IF .$err=="REACTOR_ねじ締め計測1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7307
  END
  IF .$err=="REACTOR_ねじ締め計測2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7308
  END
  IF .$err=="REACTOR_ねじ締め計測3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7309
  END
  IF .$err=="TOPRACK_ねじ締め計測1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7310
  END
  IF .$err=="TOPRACK_ねじ締め計測2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7311
  END
  IF .$err=="TOPRACK_ねじ締め計測3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7312
  END
  IF .$err=="GAS_ねじ締め計測1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7313
  END
  IF .$err=="GAS_ねじ締め計測2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7314
  END
  IF .$err=="GAS_ねじ締め計測3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7315
  END
;
  IF .$err=="RF_ﾕﾆｯﾄ取り外し1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7401
  END
  IF .$err=="RF_ﾕﾆｯﾄ取り外し2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7402
  END
  IF .$err=="RF_ﾕﾆｯﾄ取り外し3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7403
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ取り外し1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7404
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ取り外し2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7405
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ取り外し3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7406
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ取り外し1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7407
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ取り外し2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7408
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ取り外し3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7409
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ取り外し1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7410
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ取り外し2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7411
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ取り外し3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7412
  END
  IF .$err=="GAS_ﾕﾆｯﾄ取り外し1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7413
  END
  IF .$err=="GAS_ﾕﾆｯﾄ取り外し2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7414
  END
  IF .$err=="GAS_ﾕﾆｯﾄ取り外し3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7415
  END
;
  IF .$err=="RF_ﾕﾆｯﾄ置き台1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7501
  END
  IF .$err=="RF_ﾕﾆｯﾄ置き台2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7502
  END
  IF .$err=="RF_ﾕﾆｯﾄ置き台3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7503
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ置き台1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7504
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ置き台2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7505
  END
  IF .$err=="EXHAUST_ﾕﾆｯﾄ置き台3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7506
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ置き台1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7507
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ置き台2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7508
  END
  IF .$err=="REACTOR_ﾕﾆｯﾄ置き台3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7509
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ置き台1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7510
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ置き台2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7511
  END
  IF .$err=="TOPRACK_ﾕﾆｯﾄ置き台3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7512
  END
  IF .$err=="GAS_ﾕﾆｯﾄ置き台1点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7513
  END
  IF .$err=="GAS_ﾕﾆｯﾄ置き台2点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7514
  END
  IF .$err=="GAS_ﾕﾆｯﾄ置き台3点目_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7515
  END
;
  IF .$err=="RB歪み_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7601
  END
  IF .$err=="吊り具歪み_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7602
  END
  IF .$err=="ﾏﾃﾊﾝlong歪み_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7603
  END
  IF .$err=="ﾏﾃﾊﾝshort歪み_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7604
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1歪み_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7605
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2歪み_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7606
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20歪み_2Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7607
  END
;
  IF .$err=="M12ﾎﾞﾙﾄ在籍計測_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7701
  END
  IF .$err=="M20ﾎﾞﾙﾄ在籍計測_3Dﾋﾞｼﾞｮﾝ計測ｴﾗｰ" THEN
    err_num = 7702
  END
  IF .$err=="RF_ﾎﾞﾙﾄ在籍計測_ﾎﾞﾙﾄ本数不足ｴﾗｰ" THEN
    err_num = 7703
  END
  IF .$err=="EXHAUST_ﾎﾞﾙﾄ在籍計測_ﾎﾞﾙﾄ本数不足ｴﾗｰ" THEN
    err_num = 7704
  END
  IF .$err=="REACTOR_ﾎﾞﾙﾄ在籍計測_ﾎﾞﾙﾄ本数不足ｴﾗｰ" THEN
    err_num = 7705
  END
  IF .$err=="TOPRACK_ﾎﾞﾙﾄ在籍計測_ﾎﾞﾙﾄ本数不足ｴﾗｰ" THEN
    err_num = 7706
  END
  IF .$err=="GAS_ﾎﾞﾙﾄ在籍計測_ﾎﾞﾙﾄ本数不足ｴﾗｰ" THEN
    err_num = 7707
  END
;
  IF .$err=="ﾋﾞｼﾞｮﾝ計測結果_数値範囲外ｴﾗｰ" THEN
    err_num = 7801
  END
;
;<< ﾅｯﾄﾗﾝﾅｰ系ｴﾗｰ >>
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_ﾎﾞﾙﾄ吸着ｴﾗｰ" THEN
    err_num = 8001
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_ﾎﾞﾙﾄ吸着ｴﾗｰ" THEN
    err_num = 8002
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20_ﾎﾞﾙﾄ吸着ｴﾗｰ" THEN
    err_num = 8003
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_ﾎﾞﾙﾄ脱落ｴﾗｰ" THEN
    err_num = 8004
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_ﾎﾞﾙﾄ脱落ｴﾗｰ" THEN
    err_num = 8005
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20_ﾎﾞﾙﾄ脱落ｴﾗｰ" THEN
    err_num = 8006
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_READY未完了ｴﾗｰ" THEN
    err_num = 8007
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_READY未完了ｴﾗｰ" THEN
    err_num = 8008
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20_READY未完了ｴﾗｰ" THEN
    err_num = 8009
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_RF1本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8101
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_RF2本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8102
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_RF3本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8103
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_RF4本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8104
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_RF5本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8105
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_RF6本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8106
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_RF7本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8107
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_RF8本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8108
  END
;  
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20_EXHAUST1本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8201
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20_EXHAUST2本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8202
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20_EXHAUST3本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8203
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20_EXHAUST4本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8204
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM20_EXHAUST5本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8205
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_EXHAUST1本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8206
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_EXHAUST2本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8207
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_EXHAUST3本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8208
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_EXHAUST4本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8209
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_EXHAUST5本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8210
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_EXHAUST6本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8211
  END
;
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_REACTOR1本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8301
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_REACTOR2本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8302
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_REACTOR3本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8303
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_REACTOR4本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8304
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_REACTOR5本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8305
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_REACTOR6本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8306
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_REACTOR7本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8307
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_REACTOR8本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8308
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_REACTOR1本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8309
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_REACTOR2本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8310
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_REACTOR3本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8311
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_REACTOR4本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8312
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_REACTOR5本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8313
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_REACTOR6本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8314
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_REACTOR7本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8315
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_REACTOR8本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8316
  END
;
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_TOPRACK1本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8401
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_1_TOPRACK2本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8402
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_TOPRACK1本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8403
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_TOPRACK2本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8404
  END
;
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_GAS1本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8501
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_GAS2本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8502
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_GAS3本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8503
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_GAS4本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8504
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_GAS5本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8505
  END
  IF .$err=="ﾅｯﾄﾗﾝﾅｰM12_2_GAS6本目_ﾄﾙｸ未完了ｴﾗｰ" THEN
    err_num = 8506
  END
;
;<< 水平ﾌﾟｯｼｬｴﾗｰ >>
  IF .$err=="水平ﾌﾟｯｼｬ出端ｴﾗｰ" THEN
    err_num = 9001
  END
  IF .$err=="水平ﾌﾟｯｼｬ戻端ｴﾗｰ" THEN
    err_num = 9002
  END
;
;<< ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰｴﾗｰ >>
  IF .$err=="ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰ出端ｴﾗｰ" THEN
    err_num = 9003
  END
  IF .$err=="ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰ戻端ｴﾗｰ" THEN
    err_num = 9004
  END
;
;
;
;
  IFPWPRINT 1,1,,7,10=.$err
;
  RESET ; 信号ﾘｾｯﾄ
;
  IF 8100<err_num AND err_num<9000 THEN ; ﾄﾙｸ未完了ｴﾗｰなら
    RETURN ; 止めない
  END
;
  BITS ox_apperr_num[1],16 = err_num
  SIGNAL ox_app_error ; ｱﾌﾟﾘｴﾗｰ発生中  
  PAUSE ; ｱﾌﾟﾘｴﾗｰ停止
;
.END



;***********************************************************
;装置操作関連
;***********************************************************

.PROGRAM md_ctrl_slider(.$ope) #16; ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰ操作
; /======================================================================/
; FUNCTION: ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰ操作
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  UTIMER @timeout = 0
;
  IF .$ope=="supply" THEN
    SIGNAL ox_slider_out[1],-ox_slider_ret[1] ; ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰ進行,後退
    IF SIG(wx_slider_out[1]) AND SIG(-wx_slider_ret[1]) GOTO end ; 既に条件成立してたら終了
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error("ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰ出端ｴﾗｰ")
      END
    UNTIL SIG(wx_slider_out[1]) AND SIG(-wx_slider_ret[1]) ; 出端ON/戻端OFF      
  END
;
  IF .$ope=="return" THEN
    SIGNAL -ox_slider_out[1],ox_slider_ret[1] ; ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰ進行,後退
    IF SIG(-wx_slider_out[1]) AND SIG(wx_slider_ret[1]) GOTO end ; 既に条件成立してたら終了
    DO
      IF UTIMER(@timeout)>sys_timeout THEN
        CALL z.app_error("ﾎﾞﾙﾄ供給ｽﾗｲﾀﾞｰ戻端ｴﾗｰ")
      END
    UNTIL SIG(-wx_slider_out[1]) AND SIG(wx_slider_ret[1]) ; 出端OFF/戻端ON  
  END
;
end:
;
  RETURN
;
.END

.PROGRAM md_ctrl_push(.$ope) #12; RF/EXHﾌﾟｯｼｬｰ操作
; /======================================================================/
; FUNCTION: RF/EXHﾌﾟｯｼｬｰ操作
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
  UTIMER @timeout = 0
;
  IF .$ope=="push" THEN
    IF rb_no==1 THEN
      SIGNAL ox_pussher_out[1],-ox_pussher_ret[1] ; 水平ﾌﾟｯｼｬ(RF側)進行,水平ﾌﾟｯｼｬ(RF側)後退
      IF SIG(wx_pussher_out[1]) AND SIG(-wx_pussher_ret[1]) GOTO end ; 既に条件成立してたら終了
      DO
        IF UTIMER(@timeout)>sys_timeout THEN
          CALL z.app_error("水平ﾌﾟｯｼｬ出端ｴﾗｰ")
        END
      UNTIL SIG(wx_pussher_out[1]) AND SIG(-wx_pussher_ret[1]) ; 出端ON/戻端OFF      
    END
    IF rb_no==2 THEN
      SIGNAL ox_pussher_out[2],-ox_pussher_ret[2] ; 水平ﾌﾟｯｼｬ(EXHAUST側)進行,水平ﾌﾟｯｼｬ(EXHAUST側)後退
      IF SIG(wx_pussher_out[2]) AND SIG(-wx_pussher_ret[2]) GOTO end ; 既に条件成立してたら終了
      DO
        IF UTIMER(@timeout)>sys_timeout THEN
          CALL z.app_error("水平ﾌﾟｯｼｬ出端ｴﾗｰ")
        END
      UNTIL SIG(wx_pussher_out[2]) AND SIG(-wx_pussher_ret[2]) ; 出端ON/戻端OFF
    END
  END
;
  IF .$ope=="return" THEN
    IF rb_no==1 THEN
      SIGNAL -ox_pussher_out[1],ox_pussher_ret[1] ; 水平ﾌﾟｯｼｬ(RF側)進行,水平ﾌﾟｯｼｬ(RF側)後退
      IF SIG(-wx_pussher_out[1]) AND SIG(wx_pussher_ret[1]) GOTO end ; 既に条件成立してたら終了
      DO
        IF UTIMER(@timeout)>sys_timeout THEN
          CALL z.app_error("水平ﾌﾟｯｼｬ戻端ｴﾗｰ")
        END
      UNTIL SIG(-wx_pussher_out[1]) AND SIG(wx_pussher_ret[1]) ; 出端OFF/戻端ON  
    END
    IF rb_no==2 THEN
      SIGNAL -ox_pussher_out[2],ox_pussher_ret[2] ; 水平ﾌﾟｯｼｬ(EXHAUST側)進行,水平ﾌﾟｯｼｬ(EXHAUST側)後退
      IF SIG(-wx_pussher_out[2]) AND SIG(wx_pussher_ret[2]) GOTO end ; 既に条件成立してたら終了
      DO
        IF UTIMER(@timeout)>sys_timeout THEN
          CALL z.app_error("水平ﾌﾟｯｼｬ戻端ｴﾗｰ")
        END
      UNTIL SIG(-wx_pussher_out[2]) AND SIG(wx_pussher_ret[2]) ; 出端OFF/戻端ON  
    END
  END
;
end:
;
  RETURN
;
.END



;***********************************************************
;オートスタートPG
;***********************************************************

.PROGRAM autostart.pc() #0
; /======================================================================/
; FUNCTION: autostart.pc
; WorkType: Common
; Copyright(c)2022 by Kawasaki Robot Service,Ltd.  
; /======================================================================/
;
;takehara 起動時の初期化処理
  .zpowoff_flg = OFF
  FOR .i = 1 TO 7
    .atc_flg[.i] = OFF
  END
;
loop:
  TWAIT ascycle
;
;<< 吊り具把持中の監視 >>
;ｱｼｽﾄｸﾚｰﾝがHOLD状態になったときﾛﾎﾞｯﾄ止める
;takehara どういう状況？CSってなんだったっけ。→サイクルスタート時
  IF SWITCH(CS )==ON THEN
    IF SIG(wx_close_atc[1]) OR SIG(wx_close_atc[2]) THEN ; 吊り具把持中
      IF SIG(wx_crn_pos[3]) THEN ; ﾎｰﾙﾄﾞ中
        MC HOLD
        SIGNAL ox_app_error ; ｱﾌﾟﾘｴﾗｰ発生中
        .e_n = 3008 ; ｱｼｽﾄｸﾚｰﾝ_ｻｲｸﾙ運転中HOLDｴﾗｰ
        BITS ox_apperr_num[1],16 = .e_n
        TWAIT 0.5
      END
    END
  END
;
;<< 品種番号ｱﾝｻｰ >>
  FOR .i = 1 TO 13
    IF SIG(wx_object[.i]) THEN ; PLCからの品種番号指示をそのまま返す
      SIGNAL ox_object_ans[.i]
    ELSE
      SIGNAL -ox_object_ans[.i]
    END
  END
;
;<< PGの対象ﾕﾆｯﾄをPLCへ出力 >>
  CALL md_work_pickup
;
  IF SIG(2001) THEN ; IFP表示窓ﾘｾｯﾄ
    IFPWPRINT 1,1,,7,10=""
  END
;
  IF SIG(wx_error_reset) THEN ; 外部ｴﾗｰﾘｾｯﾄ
    SIGNAL -ox_app_error ; ｱﾌﾟﾘｴﾗｰ発生中OFF
    BITS ox_apperr_num[1],16 = 0 ; ｱﾌﾟﾘｴﾗｰ番号出力ﾘｾｯﾄ
  END
;
;takehara ここでの用途は何？→使ってないので不要
  IF SIG(ox_home1) AND SIG(-wx_close_atc[1]) AND SIG(-wx_close_atc[2]) AND SIG(-wx_close_atc[5]) AND SIG(-wx_close_atc[6]) AND SIG(-wx_close_atc[7]) THEN ; 第一原点/何もﾁｬｯｸしてない
    move_no = 0 ; 動作番号0(原点)
  END
;
;
;
;<< ATC着脱位置監視 >>
;ATC着脱位置にいるとき、取置位置到着信号をONする処理
;ﾃｨｰﾁﾓｰﾄﾞ時のみ有効(ﾘﾋﾟｰﾄ時は動作ﾌﾟﾛ内でON/OFF)
  IF SWITCH(REPEAT )==OFF THEN ; ﾘﾋﾟｰﾄでない
;
;takehara 事後防止のためにATC着脱位置付近にいるときでないとATCを操作できないようにしている。
    IF DISTANCE(pick_tsuri[1],HERE)<10 OR DISTANCE(set_tsuri[1],HERE)<10 THEN ; 吊り具上
      SIGNAL ox_arrive_atc[1]
      .atc_flg[1] = ON
    ELSE
      SIGNAL -ox_arrive_atc[1]
      .atc_flg[1] = OFF
    END
;
    IF DISTANCE(pick_tsuri[2],HERE)<10 OR DISTANCE(set_tsuri[2],HERE)<10 THEN ; 吊り具横
      SIGNAL ox_arrive_atc[2]
      .atc_flg[2] = ON
    ELSE
      SIGNAL -ox_arrive_atc[2]
      .atc_flg[2] = OFF
    END
;
    IF DISTANCE(pick_hand[1],HERE)<10 OR DISTANCE(set_hand[1],HERE)<10 THEN ; ﾏﾃﾊﾝlong
      SIGNAL ox_arrive_atc[3]
      .atc_flg[3] = ON
    ELSE
      SIGNAL -ox_arrive_atc[3]
      .atc_flg[3] = OFF
    END
;
    IF rb_no==1 THEN ; RB1
      IF DISTANCE(pick_hand[2],HERE)<10 OR DISTANCE(set_hand[2],HERE)<10 THEN ; ﾏﾃﾊﾝshort
        SIGNAL ox_arrive_atc[4]
        .atc_flg[4] = ON
      ELSE
        SIGNAL -ox_arrive_atc[4]
        .atc_flg[4] = OFF
      END
    ELSE ; RB2
;pick(set)_hand_distはRB2のshort上持ちの歪みﾁｪｯｸ用
      IF DISTANCE(pick_hand[2],HERE)<10 OR DISTANCE(set_hand[2],HERE)<10 OR DISTANCE(pick_hand_dist[2],HERE)<10 OR DISTANCE(set_hand_dist[2],HERE)<10 THEN ; ﾏﾃﾊﾝshort
        SIGNAL ox_arrive_atc[4]
        .atc_flg[4] = ON
      ELSE
        SIGNAL -ox_arrive_atc[4]
        .atc_flg[4] = OFF
      END
    END
;
    IF rb_no==1 THEN ; RB1のみ
      IF DISTANCE(pick_nat[1],HERE)<10 OR DISTANCE(set_nat[1],HERE)<10 THEN ; ﾅｯﾄﾗﾝﾅｰM12_1
        SIGNAL ox_arrive_atc[5]
        .atc_flg[5] = ON
      ELSE
        SIGNAL -ox_arrive_atc[5]
        .atc_flg[5] = OFF
      END
    END
;
    IF rb_no==2 THEN ; RB2のみ
      IF DISTANCE(pick_nat[2],HERE)<10 OR DISTANCE(set_nat[2],HERE)<10 THEN ; ﾅｯﾄﾗﾝﾅｰM12_2
        SIGNAL ox_arrive_atc[6]
        .atc_flg[6] = ON
      ELSE
        SIGNAL -ox_arrive_atc[6]
        .atc_flg[6] = OFF
      END
;
      IF DISTANCE(pick_nat[3],HERE)<10 OR DISTANCE(set_nat[3],HERE)<10 THEN ; ﾅｯﾄﾗﾝﾅｰM10
        SIGNAL ox_arrive_atc[7]
        .atc_flg[7] = ON
      ELSE
        SIGNAL -ox_arrive_atc[7]
        .atc_flg[7] = OFF
      END
    END
;
  ELSE ; ﾘﾋﾟｰﾄ
    ;takehara リピートモード中
    FOR .i = 1 TO 7
      IF .atc_flg[.i]==ON THEN ; ﾃｨｰﾁ中にONにしていた信号をOFF
        SIGNAL -ox_arrive_atc[.i]
        .atc_flg[.i] = OFF
      END
    END
;
  END
;
;<<PLC側ｴﾗｰ時のﾛﾎﾞｯﾄIﾗｰﾘｾｯﾄ処理>>
  IF (SIG(wx_zpowoff) OR SWITCH(TP_EMG )==ON OR SWITCH(OP_EMG )==ON OR SWITCH(EX_EMG )==ON) AND SWITCH(REPEAT )==ON AND .zpowoff_flg==OFF THEN
    RESET
    .zpowoff_flg = ON
  END
  IF SIG(-wx_zpowoff) AND SWITCH(TP_EMG )==OFF AND SWITCH(OP_EMG )==OFF AND SWITCH(EX_EMG )==OFF AND .zpowoff_flg==ON THEN
    .zpowoff_flg = OFF
  END
;  
;ﾂｰﾙ先端速度取得
;SYSDATA(TOOL.VEL.CMD)
;
  GOTO loop
;
.END


