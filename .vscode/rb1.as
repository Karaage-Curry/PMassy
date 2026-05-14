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