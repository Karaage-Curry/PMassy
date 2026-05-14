.PROGRAM main2()
; ============================
; ロボット② 計測プログラム
; ============================

; ---- 初期待機 ----
WAIT PLC_START = ON
MOVE HOME

; ---- 計測指令待ち ----
WAIT ROB2_MEASURE_REQ = ON  ; ロボ①からの指令

; ---- 計測開始 ----

; 正面4点
MOVE FRONT_P1
MEAS front1

MOVE FRONT_P2
MEAS front2

MOVE FRONT_P3
MEAS front3

MOVE FRONT_P4
MEAS front4

; 側面4点
MOVE SIDE_P1
MEAS side1

MOVE SIDE_P2
MEAS side2

MOVE SIDE_P3
MEAS side3

MOVE SIDE_P4
MEAS side4

; ---- 演算処理 ----
CALL CALC_OFFSET            ; 別関数（先に作成したロジック）

; ---- 判定 ----
IF MEASURE_OK == OFF THEN
    SIGNAL PLC_ERROR        ; PLCへ異常送信
    MOVE HOME
    STOP
ENDIF

; ---- データ送信 ----
SIGNAL PLC_SEND_DATA
; X_offset, Y_offset, θ, OKフラグ送信

; ---- 原点復帰 ----
MOVE HOME

; ---- 待機 ----
WAIT NEXT_CYCLE
``
.END


.PROGRAM measure()
# -------------------------
# 設定値
# -------------------------
OUTLIER_TH = 3.0
ANGLE_LIMIT = 2.0

# -------------------------
# 入力（各点バラ変数）
# -------------------------
front1 = 100.1
front2 = 100.5
front3 = 99.8
front4 = 100.2

side1 = 200.0
side2 = 199.8
side3 = 200.2
side4 = 200.1

# 基準値
z_front_ref = 100.0
z_side_ref  = 200.0

x_left  = -50.0
x_right = 50.0

y_top    = 50.0
y_bottom = -50.0

# -------------------------
# ① 平均（仮）
# -------------------------
front_avg_tmp = (front1 + front2 + front3 + front4) / 4
side_avg_tmp  = (side1 + side2 + side3 + side4) / 4

# -------------------------
# ② 外れ値判定フラグ
# -------------------------
f1_ok = abs(front1 - front_avg_tmp) <= OUTLIER_TH
f2_ok = abs(front2 - front_avg_tmp) <= OUTLIER_TH
f3_ok = abs(front3 - front_avg_tmp) <= OUTLIER_TH
f4_ok = abs(front4 - front_avg_tmp) <= OUTLIER_TH

s1_ok = abs(side1 - side_avg_tmp) <= OUTLIER_TH
s2_ok = abs(side2 - side_avg_tmp) <= OUTLIER_TH
s3_ok = abs(side3 - side_avg_tmp) <= OUTLIER_TH
s4_ok = abs(side4 - side_avg_tmp) <= OUTLIER_TH

# -------------------------
# ③ 平均再計算（手動積み上げ）
# -------------------------
front_sum = 0.0
front_cnt = 0

if f1_ok:
    front_sum += front1
    front_cnt += 1

if f2_ok:
    front_sum += front2
    front_cnt += 1

if f3_ok:
    front_sum += front3
    front_cnt += 1

if f4_ok:
    front_sum += front4
    front_cnt += 1

side_sum = 0.0
side_cnt = 0

if s1_ok:
    side_sum += side1
    side_cnt += 1

if s2_ok:
    side_sum += side2
    side_cnt += 1

if s3_ok:
    side_sum += side3
    side_cnt += 1

if s4_ok:
    side_sum += side4
    side_cnt += 1

# -------------------------
# ④ 点数チェック（最低3点）
# -------------------------
if front_cnt < 3 or side_cnt < 3:
    result_OK = False
else:
    result_OK = True

# -------------------------
# ⑤ 平均確定
# -------------------------
if result_OK:
    front_avg = front_sum / front_cnt
    side_avg  = side_sum / side_cnt
else:
    front_avg = 0
    side_avg = 0

# -------------------------
# ⑥ オフセット
# -------------------------
Y_offset = front_avg - z_front_ref
X_offset = side_avg - z_side_ref

# -------------------------
# ⑦ θ（正面）
# -------------------------
z_left  = (front1 + front3) / 2
z_right = (front2 + front4) / 2

dx = x_right - x_left

theta_front = math.degrees(math.atan((z_right - z_left) / dx))

# -------------------------
# ⑧ θ（側面）
# -------------------------
dy = y_top - y_bottom

theta_side = math.degrees(math.atan((side1 - side4) / dy))

# -------------------------
# ⑨ θチェック
# -------------------------
if abs(theta_front - theta_side) > ANGLE_LIMIT:
    result_OK = False

# -------------------------
# ⑩ 出力
# -------------------------
if result_OK:
    print("OK")
    print("X_offset =", X_offset)
    print("Y_offset =", Y_offset)
    print("Theta =", theta_front)
else:
    print("NG（エラー処理）")
    
.END

