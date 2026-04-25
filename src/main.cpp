#include "PositionPID.h"
#include "SerialConsole.h"
#include "SpeedPID.h"
#include <Arduino.h>
#include <ESP32Encoder.h>
#include <PS4Controller.h>
#include <WiFi.h>
#include <cmath>
#include <cstdio>
#include <esp_now.h>

constexpr int8_t PIN_DIR_1 = 23;
constexpr int8_t PIN_PWM_1 = 22;
constexpr int8_t PIN_DIR_2 = 21;
constexpr int8_t PIN_PWM_2 = 19;
constexpr int8_t PIN_DIR_3 = 18;
constexpr int8_t PIN_PWM_3 = 17;

constexpr int8_t PIN_ROTARY_A_1 = 27;
constexpr int8_t PIN_ROTARY_B_1 = 14;
constexpr int8_t PIN_ROTARY_A_2 = 25;
constexpr int8_t PIN_ROTARY_B_2 = 26;
constexpr int8_t PIN_ROTARY_A_3 = 32;
constexpr int8_t PIN_ROTARY_B_3 = 33;

constexpr int8_t W1_CH = 0;
constexpr int8_t W2_CH = 1;
constexpr int8_t W3_CH = 2;

constexpr int8_t W1_SIGN = 1;
constexpr int8_t W2_SIGN = 1;
constexpr int8_t W3_SIGN = 1;

constexpr double WHEEL_DIAMETER_MM  = 60;      // 車輪直径[mm]
constexpr double ENC_COUNTS_PER_REV = 4096.0;  // エンコーダ1回転カウント
constexpr double GEAR_RATIO         = 1.0;     // 減速比（モータ回転/車輪回転）
constexpr double ROBOT_RADIUS_MM    = 113.838; // 機体中心→車輪接地点の半径[mm]

constexpr double COUNTS_PER_MM = (ENC_COUNTS_PER_REV * GEAR_RATIO) / (PI * WHEEL_DIAMETER_MM);

ESP32Encoder enc1, enc2, enc3;

double wheel_v1 = 0.0, wheel_v2 = 0.0, wheel_v3 = 0.0;
double wheel_target_1 = 0.0, wheel_target_2 = 0.0, wheel_target_3 = 0.0;

double x_mm         = 0.0;
double y_mm         = 0.0;
double theta        = 0.0; // 追加: 姿勢角[rad]
double target_x_mm  = 0;
double target_y_mm  = 0;
double target_theta = 0;
bool   moving       = false;

constexpr double  INTEGRAL_MAX          = 10.0; // int8_t -> double
constexpr int16_t PWM_LIMIT             = 255;
constexpr double  MAX_VX_CMD_MM_S       = 300.0;
constexpr double  MAX_VY_CMD_MM_S       = 300.0; // 追加
constexpr double  NEAR_SPEED_LIMIT_MM_S = 80.0;
constexpr double  POSITION_DEADBAND_MM  = 3.0; // 1.0 -> 3.0
constexpr double  SPEED_DEADBAND_MM_S   = 2.0;
constexpr double  MAX_OMEGA_CMD_RAD_S   = 2.0;  // 追加: 角速度上限[rad/s]
constexpr double  THETA_DEADBAND_RAD    = 0.08; // 0.03 -> 0.08
constexpr int     PWM_DEADBAND          = 8;
constexpr double  DEG2RAD               = PI / 180.0;

const double RAD2DEG = 360 / (2 * PI);

constexpr double USER_SIGN                = -1.0;
volatile int16_t rx_target_x_mm_i16       = 0;
volatile int16_t rx_target_y_mm_i16       = 0;
volatile int16_t rx_target_theta_mrad_i16 = 0;
volatile bool    rx_new_target            = false;

PositionPid position_pid_x(1.5, 0.0, 0.01, -MAX_VX_CMD_MM_S, MAX_VX_CMD_MM_S, -INTEGRAL_MAX, INTEGRAL_MAX);
PositionPid position_pid_y(1.5, 0.0, 0.01, -MAX_VY_CMD_MM_S, MAX_VY_CMD_MM_S, -INTEGRAL_MAX, INTEGRAL_MAX);
PositionPid position_pid_theta(2.0, 0.0, 0.01, -MAX_OMEGA_CMD_RAD_S, MAX_OMEGA_CMD_RAD_S, -INTEGRAL_MAX, INTEGRAL_MAX);

SpeedPID speed_pid_1(3., 7., 0.00, -PWM_LIMIT, PWM_LIMIT);
SpeedPID speed_pid_2(3., 7., 0.00, -PWM_LIMIT, PWM_LIMIT);
SpeedPID speed_pid_3(3., 7., 0.00, -PWM_LIMIT, PWM_LIMIT);

constexpr int8_t ENCODER_SIGN_1 = -1;
constexpr int8_t ENCODER_SIGN_2 = -1;
constexpr int8_t ENCODER_SIGN_3 = -1;

double wrapPi(double rad) {
    while (rad > PI)
        rad -= 2.0 * PI;
    while (rad < -PI)
        rad += 2.0 * PI;
    return rad;
}

// 各車輪の「移動量(mm)」から本体の差分(dx,dy,dtheta)を計算
void wheelDeltaToBodyDelta(double s1, double s2, double s3, double& dx, double& dy, double& dtheta) {
    dx     = (2.0 * s1 - s2 - s3) / 3.0;
    dy     = (s2 - s3) / sqrt(3.0);
    dtheta = (s1 + s2 + s3) / (3.0 * ROBOT_RADIUS_MM);
}

// 本体の動きから各車輪速度
void bodyToWheel(double vx, double vy, double omega, double& w1, double& w2, double& w3) {
    // vxの影響をそのまま受けて、回転成分が加わる
    w1 = 1.0 * vx + ROBOT_RADIUS_MM * omega;
    w2 = -0.5 * vx + 0.5 * sqrt(3) * vy + ROBOT_RADIUS_MM * omega;
    w3 = -0.5 * vx - 0.5 * sqrt(3) * vy + ROBOT_RADIUS_MM * omega;
}

// 3輪エンコーダ差分カウント -> 本体差分移動量
void countsToBodyDelta(long dc1, long dc2, long dc3, double& dx, double& dy, double& dtheta) {
    const double s1 = (double)(dc1) / COUNTS_PER_MM;
    const double s2 = (double)(dc2) / COUNTS_PER_MM;
    const double s3 = (double)(dc3) / COUNTS_PER_MM;

    wheelDeltaToBodyDelta(s1, s2, s3, dx, dy, dtheta);
}

// signは、モータの配線や取り付け向きの補正
void setMotor(int8_t dirPin, int pwmCh, int sign, int pwm_signed) {
    int duty = sign * pwm_signed;
    if (abs(duty) < PWM_DEADBAND) {
        ledcWrite(pwmCh, 0);
        return; // 0付近はDIRを触らない
    }
    digitalWrite(dirPin, (duty > 0) ? HIGH : LOW);
    ledcWrite(pwmCh, abs(duty));
}
static inline double clampd(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}
unsigned long last = micros();

// 送信先の情報を保持する構造体
// ブロードキャスト宛先を入れるために使っている
esp_now_peer_info_t slave;

// 送信コールバック
void OnDataSent(const uint8_t* mac_addr, esp_now_send_status_t status) {
    // 送信先のmacアドレス
    char macStr[18];
    // snprintf、macアドレス6バイトを人間が読める形式に変換する
    snprintf(macStr, sizeof(macStr), "%02X:%02X:%02X:%02X:%02X:%02X", mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3],
             mac_addr[4], mac_addr[5]);
}

// 受信コールバック
void OnDataRecv(const uint8_t* mac_addr, const uint8_t* data, int data_len) {
    if (data_len < 6) return;

    const int16_t rx_x          = (int16_t)((data[0] << 8) | data[1]);
    const int16_t rx_y          = (int16_t)((data[2] << 8) | data[3]);
    const int16_t rx_theta_mrad = (int16_t)((data[4] << 8) | data[5]);

    rx_target_x_mm_i16       = rx_x;
    rx_target_y_mm_i16       = rx_y;
    rx_target_theta_mrad_i16 = rx_theta_mrad;
    rx_new_target            = true;
}

void setup() {
    Serial.begin(115200);
    // PS4.begin("48:e7:29:a3:b2:26");
    // Serial.println("Ready.");

    pinMode(PIN_DIR_1, OUTPUT);
    pinMode(PIN_DIR_2, OUTPUT);
    pinMode(PIN_DIR_3, OUTPUT);

    ESP32Encoder::useInternalWeakPullResistors = puType::up;
    enc1.attachHalfQuad(PIN_ROTARY_A_1, PIN_ROTARY_B_1);
    enc2.attachHalfQuad(PIN_ROTARY_A_2, PIN_ROTARY_B_2);
    enc3.attachHalfQuad(PIN_ROTARY_A_3, PIN_ROTARY_B_3);

    enc1.clearCount();
    enc2.clearCount();
    enc3.clearCount();

    ledcSetup(0, 12800, 8);
    ledcSetup(1, 12800, 8);
    ledcSetup(2, 12800, 8);
    // ledcSetup(3, 12800, 8);
    // ledcSetup(4, 12800, 8);
    ledcAttachPin(PIN_PWM_1, 0);
    ledcAttachPin(PIN_PWM_2, 1);
    ledcAttachPin(PIN_PWM_3, 2);

    // ESP-NOW初期化
    WiFi.mode(WIFI_STA);
    WiFi.disconnect();

    if (esp_now_init() == ESP_OK) {
        Serial.println("ESPNow Init Success");
    } else {
        Serial.println("ESPNow Init Failed");
        ESP.restart();
    }

    // マルチキャスト用Slave登録
    // 構造体を0クリア
    memset(&slave, 0, sizeof(slave));

    // 送信先MACアドレスを設定
    // E4:65:B8:7E:0F:F0
    uint8_t peerAddress[] = {0xE4, 0x65, 0xB8, 0x7E, 0x0F, 0xF0};
    memcpy(slave.peer_addr, peerAddress, 6);

    // チャンネル指定(0はチャンネル自動)
    slave.channel = 0;
    // 暗号化しない
    slave.encrypt = false;

    esp_err_t addStatus = esp_now_add_peer(&slave);
    if (addStatus == ESP_OK) {
        // Pair success
        Serial.println("Pair success");
    }
    // ESP-NOWコールバック登録
    esp_now_register_send_cb(OnDataSent);
    esp_now_register_recv_cb(OnDataRecv);

    last = micros();
}

constexpr long CONTROL_CYCLE = 5000;

double dx, dy, dtheta;

long prev_count_1 = 0;
long prev_count_2 = 0;
long prev_count_3 = 0;

double w1_control;
double w2_control;
double w3_control;

void loop() {
    unsigned long now = micros();

    if (now - last < CONTROL_CYCLE) return;
    double dt = (now - last) * 1.e-6;
    last      = now;

    // 新目標の反映はloop側で行う
    if (rx_new_target) {
        const int16_t tx = rx_target_x_mm_i16;
        const int16_t ty = rx_target_y_mm_i16;
        const int16_t tt = rx_target_theta_mrad_i16;
        rx_new_target    = false;

        target_x_mm  = (double)tx;
        target_y_mm  = (double)ty;
        target_theta = wrapPi((double)tt / 1000.0);

        // 命令切替時の荒れを抑える
        position_pid_x.reset(x_mm);
        position_pid_y.reset(y_mm);
        position_pid_theta.reset(theta);

        moving = true;
    }

    long c1 = enc1.getCount();
    long c2 = enc2.getCount();
    long c3 = enc3.getCount();

    const long dc1_raw = c1 - prev_count_1;
    const long dc2_raw = c2 - prev_count_2;
    const long dc3_raw = c3 - prev_count_3;

    prev_count_1 = c1;
    prev_count_2 = c2;
    prev_count_3 = c3;

    // エンコーダ符号補正（重要）
    const long dc1 = ENCODER_SIGN_1 * dc1_raw;
    const long dc2 = ENCODER_SIGN_2 * dc2_raw;
    const long dc3 = ENCODER_SIGN_3 * dc3_raw;

    wheel_v1 = ((double)(dc1) / COUNTS_PER_MM) / dt;
    wheel_v2 = ((double)(dc2) / COUNTS_PER_MM) / dt;
    wheel_v3 = ((double)(dc3) / COUNTS_PER_MM) / dt;

    countsToBodyDelta(dc1, dc2, dc3, dx, dy, dtheta);

    // body差分 -> world差分（中点法）
    const double th_mid = theta + 0.5 * dtheta;
    const double ct_mid = cos(th_mid);
    const double st_mid = sin(th_mid);

    x_mm += ct_mid * dx - st_mid * dy;
    y_mm += st_mid * dx + ct_mid * dy;
    theta = wrapPi(theta + dtheta);

    double x_control     = 0.0;
    double y_control     = 0.0;
    double omega_control = 0.0;

    if (moving) {
        // PIDはworld系で計算
        const double x_control_w = position_pid_x.update(target_x_mm, x_mm, dt);
        const double y_control_w = position_pid_y.update(target_y_mm, y_mm, dt);

        // 角度誤差は wrap してからPIDへ
        const double th_err = wrapPi(target_theta - theta);
        const double th_ref = theta + th_err; // 連続な参照角
        omega_control       = position_pid_theta.update(th_ref, theta, dt);

        x_control = x_control_w;
        y_control = y_control_w;

        // 到達判定
        if (fabs(target_x_mm - x_mm) < POSITION_DEADBAND_MM && fabs(target_y_mm - y_mm) < POSITION_DEADBAND_MM &&
            fabs(th_err) < THETA_DEADBAND_RAD) {
            moving = false;
        }
    }

    x_control     = clampd(x_control, -MAX_VX_CMD_MM_S, MAX_VX_CMD_MM_S);
    y_control     = clampd(y_control, -MAX_VY_CMD_MM_S, MAX_VY_CMD_MM_S);
    omega_control = clampd(omega_control, -MAX_OMEGA_CMD_RAD_S, MAX_OMEGA_CMD_RAD_S);

    theta = wrapPi(theta);

    const double c       = cos(theta);
    const double s       = sin(theta);
    const double robot_x = c * x_control + s * y_control;
    const double robot_y = -s * x_control + c * y_control;

    bodyToWheel(robot_x, robot_y, omega_control, wheel_target_1, wheel_target_2, wheel_target_3);

    if (!moving) {
        wheel_target_1 = 0.0;
        wheel_target_2 = 0.0;
        wheel_target_3 = 0.0;

        speed_pid_1.reset();
        speed_pid_2.reset();
        speed_pid_3.reset();

        position_pid_x.reset(x_mm);
        position_pid_y.reset(y_mm);
        position_pid_theta.reset(theta);

        // 停止中はPID出力を使わず、完全停止を指示
        setMotor(PIN_DIR_1, W1_CH, W1_SIGN, 0);
        setMotor(PIN_DIR_2, W2_CH, W2_SIGN, 0);
        setMotor(PIN_DIR_3, W3_CH, W3_SIGN, 0);

        Serial.printf("x:%f y:%f th:%f t_x:%f t_y:%f t_th:%f\r\n", x_mm * USER_SIGN, y_mm * USER_SIGN, theta * RAD2DEG,
                      target_x_mm * USER_SIGN, target_y_mm * USER_SIGN, target_theta * RAD2DEG);
        return;
    }

    const double pwm1 = speed_pid_1.update(wheel_target_1, wheel_v1, dt);
    const double pwm2 = speed_pid_2.update(wheel_target_2, wheel_v2, dt);
    const double pwm3 = speed_pid_3.update(wheel_target_3, wheel_v3, dt);

    setMotor(PIN_DIR_1, W1_CH, W1_SIGN, lround(pwm1));
    setMotor(PIN_DIR_2, W2_CH, W2_SIGN, lround(pwm2));
    setMotor(PIN_DIR_3, W3_CH, W3_SIGN, lround(pwm3));

    int16_t send_x_mm       = (int16_t)lround(x_mm * USER_SIGN);
    int16_t send_y_mm       = (int16_t)lround(y_mm * USER_SIGN);
    int16_t send_theta_mrad = (int16_t)lround(theta * 1000.0);

    uint8_t data[9] = {(uint8_t)((send_x_mm >> 8) & 0xFF),
                       (uint8_t)(send_x_mm & 0xFF),
                       (uint8_t)((send_y_mm >> 8) & 0xFF),
                       (uint8_t)(send_y_mm & 0xFF),
                       (uint8_t)((send_theta_mrad >> 8) & 0xFF),
                       (uint8_t)(send_theta_mrad & 0xFF),
                       (uint8_t)(pwm1),
                       (uint8_t)(pwm2),
                       (uint8_t)(pwm3)};

    esp_err_t result = esp_now_send(slave.peer_addr, data, sizeof(data));

    // Serial.printf("p1:%f p2:%f p3:%f\n", pwm1, pwm2, pwm3);
    // Serial.printf("x:%f y:%f th:%f t_x:%f t_y:%f t_th:%f\r\n", x_mm * USER_SIGN, y_mm * USER_SIGN, theta * RAD2DEG,
    //   target_x_mm * USER_SIGN, target_y_mm * USER_SIGN, target_theta * RAD2DEG);
    // Serial.printf("c1:%d c2:%d c3:%d\r\n", c1, c2, c3);
    // Serial.printf("x_c:%f y_c:%f\n", x_control, y_control);
    // Serial.printf("w1_t:%f w2_t:%f w3_t:%f\n", wheel_target_1, wheel_target_2, wheel_target_3);
}