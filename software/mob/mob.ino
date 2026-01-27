#include <SPI.h>
#include <WiFi.h>
#include <atomic>
#include <esp_task_wdt.h>  // 追加
#include "spi_manager.h"
#include "adc.h"
#include "led.h"
#include "wall_sensor.h"
#include "imu.h"
#include "motor.h"
#include "encoder.h"
#include "sensors.h"
#include "motion_controller.h"
#include <math.h>

// Target wheel speed [m/s] (updated from MOT command via cmd_queue)
static float target_vr_mps = 0.0f;
static float target_vl_mps = 0.0f;

// 前進コマンド（プロファイル走行）状態
static bool fwd_active = false;
static float fwd_v_cmd_mmps = 0.0f;     // 現在指令速度 [mm/s]
static float fwd_v_target_mmps = 0.0f;  // 目標巡航速度 [mm/s]
static float fwd_a_mmps2 = 0.0f;        // 加速度 [mm/s^2]（符号付き）
static float fwd_goal_dist_mm = 0.0f;   // 絶対目標距離 [mm]（Sensors::get_distance()基準）
static float fwd_target_angle_rad = 0.0f; // 目標角度 [rad] （角度フィードバック用）
static float cumulative_goal_dist_mm = 0.0f; // 累積目標距離 [mm]（RDSTでリセット）

// 停止コマンド（減速して停止する）状態
static bool stop_active = false;
static float stop_v_cmd_mmps = 0.0f;      // 現在指令速度 [mm/s]
static float stop_v_target_mmps = 0.0f;   // 目標速度（通常 0）[mm/s]
static float stop_a_mmps2 = 0.0f;         // 減速度 [mm/s^2]（負ではなく絶対値として扱う）
static float stop_goal_dist_mm = 0.0f;    // 絶対目標距離 [mm]
static float stop_target_angle_rad = 0.0f; // 目標角度 [rad] （角度フィードバック用）
static float stop_cruise_mmps = 0.0f;     // STOP引数speed_mmps（巡航速度想定）[mm/s]

static constexpr float FINAL_APPROACH_SPEED_MMPS = 50.0f;  // STOP時の最終進入速度
static constexpr float STOP_MIN_SPEED_MMPS = 20.0f;        // STOP時の最低速度 [mm/s]

// 旋回コマンド（その場旋回）状態
static bool turn_active = false;
static float turn_target_rad = 0.0f;
static float turn_start_angle_rad = 0.0f;
static float turn_goal_angle_rad = 0.0f;
static float turn_speed_cmd_mps = 0.0f; // 指令速度（加減速制限後）

// ジャイロキャリブレーション状態
static bool gyro_calib_done_pending = false;  // キャリブレーション完了時にDONEを返す

// 角度制御パラメータ（簡易P + 速度制限）
static constexpr float TURN_KP_MPS_PER_RAD = 0.35f;   // [m/s]/rad
static constexpr float TURN_MAX_SPEED_MPS = 0.25f;
static constexpr float TURN_MIN_SPEED_MPS = 0.08f;
static constexpr float TURN_DONE_TOL_RAD = 0.03f;     // 約1.7deg
static constexpr float TURN_ACCEL_MPS2 = 1.2f;        // 旋回時の車輪速度加速度制限 [m/s^2]

// 直進時の角度フィードバックゲイン
static constexpr float ANGLE_FB_GAIN = 0.5f;  // [m/s]/rad

static inline float slew_rate_limit(float current, float target, float max_delta) {
    if (target > current + max_delta) return current + max_delta;
    if (target < current - max_delta) return current - max_delta;
    return target;
}

// グローバルなクラスは、上から順に初期化される
ADC adc(get_shared_spi());
LED led;
WallSensor wall_sensor(adc);
IMU imu(get_shared_spi());
Motor motor;
Encoder encoder;
Sensors sensors(imu, wall_sensor, adc, encoder);
MotionController motion(motor, sensors);

hw_timer_t* high_speed_timer = NULL;
std::atomic<uint32_t> timer_ticks(0);

QueueHandle_t cmd_queue; // コマンドキュー
QueueHandle_t msg_queue; // Core0 -> Core1 メッセージキュー（Serial出力用）

// ---- Core1(loop) -> Core0(RealtimeTask) コマンド定義 ----
struct SetMotorSpeedCommand {
    int16_t right_speed; // mm/s（互換）
    int16_t left_speed;  // mm/s（互換）
};

struct ForwardCommand {
    float speed_mmps;
    float accel_mmps2;
    float distance_mm;
};

struct StopCommand {
    float speed_mmps;
    float accel_mmps2;
    float distance_mm;
};

enum CommandID : uint8_t {
    CMD_SET_MOTOR_SPEED = 0x01,
    CMD_FORWARD = 0x04,
    CMD_STOP = 0x05,
    CMD_RESET_DISTANCE = 0x06,
    CMD_RESET_ANGLE = 0x07,
    CMD_TURN = 0x08,
    CMD_GYRO_CALIBRATE = 0x09,
};

struct Command {
    CommandID cmd_id;
    union {
        SetMotorSpeedCommand set_motor_speed;
        ForwardCommand forward;
        StopCommand stop;
        float turn_target_rad;
    } parameter;
};

// ---- Core0 -> Core1 メッセージ定義（Serial出力用） ----
// Core0->Core1で渡すメッセージ（固定長; 末尾は必ず\0終端）
struct MsgLine {
    char text[64];
};

static inline void enqueue_msg_line(const char* s) {
    if (!msg_queue || !s) return;
    MsgLine m;
    size_t i = 0;
    bool truncated = false;
    for (; i < sizeof(m.text) - 1 && s[i] != '\0'; ++i) {
        m.text[i] = s[i];
    }
    // メッセージが長すぎる場合の検出
    if (s[i] != '\0' && i == sizeof(m.text) - 1) {
        truncated = true;
    }
    m.text[i] = '\0';
    // DONEなど重要なメッセージを確実に送るため、最大10msまで待つ
    (void)xQueueSend(msg_queue, &m, pdMS_TO_TICKS(10));
    
    // 切り捨てが発生した場合は警告を別途送信
    if (truncated) {
        MsgLine warn;
        snprintf(warn.text, sizeof(warn.text), "#WARN: msg truncated\n");
        (void)xQueueSend(msg_queue, &warn, 0);
    }
}

void IRAM_ATTR onHighSpeedTimer() {
    timer_ticks.fetch_add(1, std::memory_order_relaxed);
}

uint32_t waitTick(uint32_t &last_tick) {
    uint32_t current_tick = timer_ticks.load(std::memory_order_relaxed);
    while(current_tick == last_tick) {
        // 待機
        asm volatile("nop");
        current_tick = timer_ticks.load(std::memory_order_relaxed);
    }
    uint32_t delta = current_tick - last_tick;
    last_tick = current_tick;
    return delta;
}

// コマンド処理・モーション更新関数の前方宣言
void handleSetMotorSpeedCommand(const SetMotorSpeedCommand& cmd);
void handleForwardCommand(const ForwardCommand& cmd);
void handleStopCommand(const StopCommand& cmd);
void handleTurnCommand(float turn_target_rad);
void processCommandQueue();
void updateForward(float dt_s);
void updateStop(float dt_s);
void updateTurn(float dt_s);

// ========================================
// コマンド処理関数の実装
// ========================================

void handleSetMotorSpeedCommand(const SetMotorSpeedCommand& cmd) {
    const float vr = static_cast<float>(cmd.right_speed) / 1000.0f;
    const float vl = static_cast<float>(cmd.left_speed) / 1000.0f;

    // 手動MOTが来たらプロファイルは停止（競合回避）
    fwd_active = false;
    stop_active = false;
    turn_active = false;

    target_vr_mps = vr;
    target_vl_mps = vl;
    motion.forward((vr + vl) * 0.5f, 0.0f);
}

void handleForwardCommand(const ForwardCommand& cmd) {
    // FWD: 加速して指定速度へ、距離到達でDONE（停止しない）
    led.red_on();  // FWD開始時に赤色LED点灯
    fwd_active = true;
    stop_active = false;
    turn_active = false;

    fwd_v_target_mmps = cmd.speed_mmps;
    fwd_a_mmps2 = cmd.accel_mmps2;

    // 累積目標距離に追加（オーバーシュートの影響を受けない）
    cumulative_goal_dist_mm += cmd.distance_mm;
    fwd_goal_dist_mm = cumulative_goal_dist_mm;
    
    // 目標角度を現在の角度に設定（まっすぐ進む）
    fwd_target_angle_rad = sensors.get_angle();

    // FWDコマンドの詳細情報を通知
    const float now_dist = sensors.get_distance();
    char msg[64];
    snprintf(msg, sizeof(msg), "#FWD: %.2f+%.2f->%.2f\n", now_dist, cmd.distance_mm, fwd_goal_dist_mm);
    enqueue_msg_line(msg);

    // 現在の指令速度（左右平均）を初期値に
    fwd_v_cmd_mmps = ((target_vr_mps + target_vl_mps) * 0.5f) * 1000.0f;

    // 目標が今より遅いなら減速方向に
    if (fwd_v_target_mmps < fwd_v_cmd_mmps && fwd_a_mmps2 > 0) {
        fwd_a_mmps2 = -fwd_a_mmps2;
    }

    if (fwd_a_mmps2 == 0.0f) {
        fwd_v_cmd_mmps = fwd_v_target_mmps;
    }
}

void handleStopCommand(const StopCommand& cmd) {
    // STOP: 指定距離で停止（必要に応じて50mm/sまで減速して進入）
    stop_active = true;
    fwd_active = false;
    turn_active = false;

    // 引数の速度は「現在速度の想定」だが、ここでは現在の指令速度も併用
    stop_v_target_mmps = 0.0f;
    stop_a_mmps2 = fabsf(cmd.accel_mmps2);
    stop_cruise_mmps = fabsf(cmd.speed_mmps);

    // 累積目標距離に追加（オーバーシュートの影響を受けない）
    cumulative_goal_dist_mm += cmd.distance_mm;
    stop_goal_dist_mm = cumulative_goal_dist_mm;
    
    // 目標角度を現在の角度に設定（まっすぐ進む）
    stop_target_angle_rad = sensors.get_angle();
    
    // STOPコマンドの詳細情報を通知
    const float now_dist = sensors.get_distance();
    char msg[64];
    snprintf(msg, sizeof(msg), "#STOP: %.2f+%.2f->%.2f\n", now_dist, cmd.distance_mm, stop_goal_dist_mm);
    enqueue_msg_line(msg);

    // 初期速度: 現在指令速度を優先
    stop_v_cmd_mmps = ((target_vr_mps + target_vl_mps) * 0.5f) * 1000.0f;
    if (stop_v_cmd_mmps <= 0.0f) {
        stop_v_cmd_mmps = stop_cruise_mmps;
    }
}

void handleTurnCommand(float target_rad) {
    // TURN: その場旋回（角度のみ）
    turn_active = true;
    turn_target_rad = target_rad;

    // 競合回避: ほかのプロファイルを停止
    fwd_active = false;
    stop_active = false;

    // 角度制御の基準を確定
    turn_start_angle_rad = sensors.get_angle();
    turn_goal_angle_rad = turn_start_angle_rad + turn_target_rad;
    turn_speed_cmd_mps = 0.0f;

    // 回り始めは MotionController の内部turn状態を新規にするため stop() しておく
    motion.stop();
}

void processCommandQueue() {
    Command q;
    while (xQueueReceive(cmd_queue, &q, 0) == pdTRUE) {
        switch (q.cmd_id) {
            case CMD_SET_MOTOR_SPEED:
                handleSetMotorSpeedCommand(q.parameter.set_motor_speed);
                break;

            case CMD_FORWARD:
                handleForwardCommand(q.parameter.forward);
                break;

            case CMD_STOP:
                handleStopCommand(q.parameter.stop);
                break;

            case CMD_RESET_DISTANCE:
                sensors.reset_distance();
                cumulative_goal_dist_mm = 0.0f;  // 累積目標距離もリセット
                enqueue_msg_line("#distance reset\n");
                enqueue_msg_line("DONE\n");
                break;

            case CMD_RESET_ANGLE:
                sensors.reset_angle();
                enqueue_msg_line("#angle reset\n");
                enqueue_msg_line("DONE\n");
                break;

            case CMD_TURN:
                handleTurnCommand(q.parameter.turn_target_rad);
                break;

            case CMD_GYRO_CALIBRATE:
                // ジャイロキャリブレーション開始（非ブロッキング）
                enqueue_msg_line("#Gyro calibration start...\n");
                sensors.calibrate_gyro();
                gyro_calib_done_pending = true;
                break;

            default:
                break;
        }
    }
}

// ========================================
// モーション状態更新関数の実装
// ========================================

void updateForward(float dt_s) {
    if (!fwd_active) return;

    const float now_dist = sensors.get_distance();
    const float remain_mm = fwd_goal_dist_mm - now_dist;

    if (remain_mm <= 0.0f) {
        fwd_active = false;
        led.red_off();  // FWD完了時に赤色LED消灯
        enqueue_msg_line("DONE\n");
    } else {
        float v_next = fwd_v_cmd_mmps;
        const float a_mag = fabsf(fwd_a_mmps2);

        if (a_mag > 1e-3f) {
            if (fwd_v_target_mmps > fwd_v_cmd_mmps) {
                v_next = fwd_v_cmd_mmps + a_mag * dt_s;
                if (v_next > fwd_v_target_mmps) v_next = fwd_v_target_mmps;
            } else if (fwd_v_target_mmps < fwd_v_cmd_mmps) {
                v_next = fwd_v_cmd_mmps - a_mag * dt_s;
                if (v_next < fwd_v_target_mmps) v_next = fwd_v_target_mmps;
            }
        } else {
            v_next = fwd_v_target_mmps;
        }

        fwd_v_cmd_mmps = v_next;
        const float v_cmd_mps = fwd_v_cmd_mmps / 1000.0f;
        
        // 角度フィードバック: 現在角度と目標角度の差分
        const float angle_error = sensors.get_angle() - fwd_target_angle_rad;
        const float lateral_correction = ANGLE_FB_GAIN * angle_error;
        
        target_vr_mps = v_cmd_mps;
        target_vl_mps = v_cmd_mps;
        motion.forward(v_cmd_mps, lateral_correction);
    }
}

void updateStop(float dt_s) {
    if (!stop_active) return;

    const float now_dist = sensors.get_distance();
    const float remain_mm = stop_goal_dist_mm - now_dist;

    // 目標距離に到達したら、まず停止指令を出し、次のループでDONEを返す
    if (remain_mm <= 0.0f) {
        stop_active = false;
        stop_v_cmd_mmps = 0.0f;
        target_vr_mps = 0.0f;
        target_vl_mps = 0.0f;
        motion.stop();
        enqueue_msg_line("DONE\n");
    } else {
        const float a_mag = stop_a_mmps2;
        const float v = stop_v_cmd_mmps;

        // まず 50mm/s まで減速する必要があるかを判断
        float dist_to_50 = 0.0f;
        if (a_mag > 1e-3f) {
            const float dv2 = (v * v) - (FINAL_APPROACH_SPEED_MMPS * FINAL_APPROACH_SPEED_MMPS);
            dist_to_50 = dv2 > 0.0f ? (dv2 / (2.0f * a_mag)) : 0.0f;
        }

        // 次に 50mm/s から 0 まで止めるのに必要な距離
        float dist_50_to_0 = 0.0f;
        if (a_mag > 1e-3f) {
            dist_50_to_0 = (FINAL_APPROACH_SPEED_MMPS * FINAL_APPROACH_SPEED_MMPS) / (2.0f * a_mag);
        }

        float v_next = v;

        // 残距離が「50->0の距離」以下なら、優先して停止に向けて減速（ただし20mm/s未満にはしない）
        if (remain_mm <= dist_50_to_0) {
            if (a_mag > 1e-3f) v_next = v - a_mag * dt_s;
            if (v_next < STOP_MIN_SPEED_MMPS) v_next = STOP_MIN_SPEED_MMPS;
        } else if (remain_mm <= (dist_to_50 + dist_50_to_0)) {
            // そろそろ 50mm/s まで落とすフェーズ
            if (a_mag > 1e-3f) v_next = v - a_mag * dt_s;
            if (v_next < FINAL_APPROACH_SPEED_MMPS) v_next = FINAL_APPROACH_SPEED_MMPS;
        } else {
            // まだ余裕がある: 今の速度維持（必要なら指定速度へ合わせる）
            // STOPの引数 speed_mmps は「巡航速度想定」なので、現在がそれ以下なら軽く合わせる
            const float cruise = stop_cruise_mmps;
            if (cruise > 0.0f && v < cruise && a_mag > 1e-3f) {
                v_next = v + a_mag * dt_s;
                if (v_next > cruise) v_next = cruise;
            }
        }

        stop_v_cmd_mmps = v_next;
        const float v_cmd_mps = stop_v_cmd_mmps / 1000.0f;

        // 角度フィードバック: 現在角度と目標角度の差分
        const float angle_error = sensors.get_angle() - stop_target_angle_rad;
        const float lateral_correction = ANGLE_FB_GAIN * angle_error;

        target_vr_mps = v_cmd_mps;
        target_vl_mps = v_cmd_mps;
        if (v_cmd_mps == 0.0f) {
            motion.stop();
        } else {
            motion.forward(v_cmd_mps, lateral_correction);
        }
    }
}

void updateTurn(float dt_s) {
    if (!turn_active) return;

    const float now_ang = sensors.get_angle();
    const float err = turn_goal_angle_rad - now_ang;   // +: 左回り目標が残っている

    if (fabsf(err) <= TURN_DONE_TOL_RAD) {
        turn_active = false;
        turn_speed_cmd_mps = 0.0f;
        target_vr_mps = 0.0f;
        target_vl_mps = 0.0f;
        motion.stop();
        enqueue_msg_line("DONE\n");
    } else {
        // P制御で「理想速度」を作る（残角が小さいほど遅く）
        float v_target = TURN_KP_MPS_PER_RAD * fabsf(err);
        if (v_target > TURN_MAX_SPEED_MPS) v_target = TURN_MAX_SPEED_MPS;
        if (v_target < TURN_MIN_SPEED_MPS) v_target = TURN_MIN_SPEED_MPS;

        // 加減速をなめらかにする（slew rate limit）
        const float dv_max = TURN_ACCEL_MPS2 * dt_s;
        turn_speed_cmd_mps = slew_rate_limit(turn_speed_cmd_mps, v_target, dv_max);

        const float target_rel = err; // 現在から見た残り角度
        (void)motion.turn_in_place(turn_speed_cmd_mps, target_rel);

        // デバッグ用
        target_vr_mps = (err >= 0.0f) ? +turn_speed_cmd_mps : -turn_speed_cmd_mps;
        target_vl_mps = (err >= 0.0f) ? -turn_speed_cmd_mps : +turn_speed_cmd_mps;
    }
}

// ========================================
// Core0 リアルタイムタスク
// ========================================

void Core0RealtimeTask(void* parameter) {
    // ハイスピードタイマー(1ms)の初期化
    high_speed_timer = timerBegin(1000); // 1000Hz = 1ms period
    timerAttachInterrupt(high_speed_timer, &onHighSpeedTimer);
    timerAlarm(high_speed_timer, 1, true, 0); // 1ms interval, auto-reload

    uint32_t last_tick = 0;
    uint32_t time_delta = 0;

    while(1){
        // 1msごとの待機
        time_delta = waitTick(last_tick);

        // センサーデータの更新
        sensors.update(time_delta);

        // コマンドキューの処理（loop() -> Core0）
        processCommandQueue();

        const float dt_s = static_cast<float>(time_delta) / 1000.0f;

        // 各モーション状態の更新
        updateForward(dt_s);
        updateStop(dt_s);
        updateTurn(dt_s);

        // ジャイロキャリブレーション完了チェック
        if (gyro_calib_done_pending && !sensors.is_calibrating()) {
            gyro_calib_done_pending = false;
            char msg[64];
            float offset = sensors.get_gyro_offset();
            snprintf(msg, sizeof(msg), "#Gyro offset=%.6f rad/s\n", offset);
            enqueue_msg_line(msg);
            enqueue_msg_line("DONE\n");
        }

        // Motion controller update (speed PID)
        motion.update(time_delta);
    }
}

void setup() {
    Serial.begin(3000000);
    delay(100);
    
    // 1. ハードウェアWDT無効化
    disableCore0WDT();
    
    // WiFi/Bluetooth無効化
    WiFi.mode(WIFI_OFF);
    btStop();

    // 共有SPIとペリフェラルの初期化
    init_shared_spi();
    motor.begin();
    encoder.begin();
    imu.begin();
    
    // 起動時に赤色LEDを消灯
    led.red_off();

    // コマンドキューの作成
    cmd_queue = xQueueCreate(16, sizeof(Command));
    if (cmd_queue == NULL) {
        Serial.printf("#Failed to create command queue!\n");
        while(1);
    }

    // Core0 -> Core1 メッセージキューの作成
    msg_queue = xQueueCreate(128, sizeof(MsgLine));
    if (msg_queue == NULL) {
        Serial.printf("#Failed to create message queue!\n");
        while(1);
    }

    // Core0でハイスピードタスクを開始
    BaseType_t result = xTaskCreatePinnedToCore(
        Core0RealtimeTask,
        "Core0RT",
        4096,
        NULL,
        configMAX_PRIORITIES - 1,  // 最高優先度
        NULL,
        0  // Core 0
    );
    
    if (result != pdPASS) {
        Serial.println("#Failed to create Core0 task!\n");
        while(1);
    }
    
    Serial.printf("#Core 0 realtime task started\n");
    Serial.printf("#System ready\n");
}
 
// Core1のループ関数
void loop() {
    // Core0からのメッセージ出力（Serial.printfはCore1側でのみ行う）
    {
        MsgLine m;
        while (msg_queue && xQueueReceive(msg_queue, &m, 0) == pdTRUE) {
            Serial.print(m.text);
        }
    }

    // センサーデータ出力は周期送信を止め、PCからの要求に応答して送る方式に変更した
    
    // UARTコマンド受信例
    if (Serial.available()) {
        // e.g. "MOT,100,100\n"
        String cmd = Serial.readStringUntil('\n');
        cmd.trim();

        if (cmd.startsWith("MOT")) {
            // モーター速度設定: MOT,100,100
            // 期待フォーマット: "MOT,<right_speed>,<left_speed>"
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1) {
                Command q;
                q.cmd_id = CMD_SET_MOTOR_SPEED;
                q.parameter.set_motor_speed.right_speed = cmd.substring(comma1 + 1, comma2).toInt();
                q.parameter.set_motor_speed.left_speed = cmd.substring(comma2 + 1).toInt();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#Motor: R=%d, L=%d\n",
                                  q.parameter.set_motor_speed.right_speed,
                                  q.parameter.set_motor_speed.left_speed);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid MOT format\n");
            }
        } else if (cmd.startsWith("WALL")) {
            // 壁センサLEDの有効/無効: WALL,1 または WALL,0
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                int en = cmd.substring(comma1 + 1).toInt();
                bool enabled = (en != 0);
                wall_sensor.set_enabled(enabled);
                Serial.printf("#WallSensor enabled=%d\n", enabled ? 1 : 0);
            } else {
                Serial.printf("#Invalid WALL format\n");
            }
        } else if (cmd.startsWith("FWD,")) {
            // FWD: FWD,<speed_mmps>,<accel_mmps2>,<distance_mm>
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            int comma3 = (comma2 >= 0) ? cmd.indexOf(',', comma2 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
                Command q;
                q.cmd_id = CMD_FORWARD;
                q.parameter.forward.speed_mmps = cmd.substring(comma1 + 1, comma2).toFloat();
                q.parameter.forward.accel_mmps2 = cmd.substring(comma2 + 1, comma3).toFloat();
                q.parameter.forward.distance_mm = cmd.substring(comma3 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#FWD speed=%.1fmm/s accel=%.1fmm/s^2 dist=%.1fmm\n",
                                  q.parameter.forward.speed_mmps,
                                  q.parameter.forward.accel_mmps2,
                                  q.parameter.forward.distance_mm);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid FWD format\n");
            }
        } else if (cmd.startsWith("STOP,")) {
            // STOP: STOP,<speed_mmps>,<accel_mmps2>,<distance_mm>
            int comma1 = cmd.indexOf(',');
            int comma2 = (comma1 >= 0) ? cmd.indexOf(',', comma1 + 1) : -1;
            int comma3 = (comma2 >= 0) ? cmd.indexOf(',', comma2 + 1) : -1;
            if (comma1 > 0 && comma2 > comma1 && comma3 > comma2) {
                Command q;
                q.cmd_id = CMD_STOP;
                q.parameter.stop.speed_mmps = cmd.substring(comma1 + 1, comma2).toFloat();
                q.parameter.stop.accel_mmps2 = cmd.substring(comma2 + 1, comma3).toFloat();
                q.parameter.stop.distance_mm = cmd.substring(comma3 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#STOP speed=%.1fmm/s accel=%.1fmm/s^2 dist=%.1fmm\n",
                                  q.parameter.stop.speed_mmps,
                                  q.parameter.stop.accel_mmps2,
                                  q.parameter.stop.distance_mm);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid STOP format\n");
            }
        } else if (cmd.startsWith("TURN,")) {
            // TURN: TURN,<angle_rad>
            int comma1 = cmd.indexOf(',');
            if (comma1 > 0) {
                Command q;
                q.cmd_id = CMD_TURN;
                q.parameter.turn_target_rad = cmd.substring(comma1 + 1).toFloat();

                if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                    Serial.printf("#TURN angle=%.4frad\n", q.parameter.turn_target_rad);
                } else {
                    Serial.printf("#Queue full!\n");
                }
            } else {
                Serial.printf("#Invalid TURN format\n");
            }
        } else if (cmd == "RDST") {
            // 距離リセット（オドメトリ）
            Command q;
            q.cmd_id = CMD_RESET_DISTANCE;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#RDST\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "RANG") {
            // 角度リセット（オドメトリ）
            Command q;
            q.cmd_id = CMD_RESET_ANGLE;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#RANG\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "GCAL") {
            // ジャイロキャリブレーション
            Command q;
            q.cmd_id = CMD_GYRO_CALIBRATE;
            if (xQueueSend(cmd_queue, &q, pdMS_TO_TICKS(10)) == pdTRUE) {
                Serial.printf("#GCAL\n");
            } else {
                Serial.printf("#Queue full!\n");
            }
        } else if (cmd == "SEN") {
            // PCからの要求に応答して1行だけSENを送る
            float gyro = sensors.get_gyro_z();      // rad/s
            float vbatt = sensors.get_battery_voltage();
            uint16_t lf = sensors.get_lf();
            uint16_t ls = sensors.get_ls();
            uint16_t rs = sensors.get_rs();
            uint16_t rf = sensors.get_rf();
            uint16_t enc_r = sensors.get_right_wheel_angle();
            uint16_t enc_l = sensors.get_left_wheel_angle();
            float odo_dist = sensors.get_distance();
            float odo_ang = sensors.get_angle();    // rad
            Serial.printf("SEN,%.2f,%.2f,%u,%u,%u,%u,%u,%u,%.2f,%.2f\n",
                          gyro, vbatt, lf, ls, rs, rf, enc_r, enc_l, odo_dist, odo_ang);
        } else {
            // デバッグ用: 不明コマンド
            Serial.printf("#Unknown cmd: %s\n", cmd.c_str());
        }
    }
    
    delay(1);
}
