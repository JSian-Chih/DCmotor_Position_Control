/*
Test of DC Motor Module for Applied Control
with different discred time method

Chao-Hsien Chih
2021-11-21
Final report: 2022-1-10
*/

#include <Arduino.h>
#include <LS7366R.h> // 讀取 QEI 解出來的編碼器位置
#include <Encoder.h> // 解轉盤的編碼器位置

#define ENC_LEFT 10
#define ENC_RIGHT 10
LS7366R enc(ENC_LEFT, ENC_LEFT, MDR0_CONF, MDR1_CONF); // 它可以同時解兩個 Encoder，只需要一個腳位

#define MOTOR_PWM_CW 22      // 馬達正轉的PWM腳位
#define MOTOR_PWM_CCW 23     // 馬達負轉的PWM腳位
#define MOTOR_PWM_FREQ 17000 // 馬達PWM命令的更新頻率 (Frequency)
#define PWM_WRITE_RES 11     // 馬達PWM寫入的解析度 (Resolution)

#define SWITCH_BUTTON 9 // 轉盤按鈕的接角
#define WHEEL_A 14      // 飛梭旋鈕的 A B Phase
#define WHEEL_B 15
Encoder Wheel_Enc(WHEEL_A, WHEEL_B);

// Variable Declaration
long int pos_1 = 0, pos_2 = 0;
long int pos = 0, pos_pre = 0;
double vel = 0, vel_pre = 0;
double vel_diff;
double u_pre = 0.0, u_pre2 = 0.0;

double e1_pre = 0.0,e1_pre2 = 0.0;

// double vel_d = 0;

int Motor_PWM_Cmd = 0;
int Motor_PWM_Max = (int)(pow(2.0, (double)(PWM_WRITE_RES))) - 1;

#define ISR_TIME 1000
IntervalTimer ISR_Timer;
double dt = (double)(ISR_TIME) / 1000000.0;
volatile bool ISR_Enable = 0;
volatile bool System_Enable = 1;

// 函數宣告
void ISR_Routine();
void ISR_Hardware_Interrupt();
void Motor_Reset();

int sign(int u_in)
{
    if (u_in > 0)
        return 1;
    else if (u_in < 0)
        return -1;
    return 0;
}

double sign(double u_in)
{
    if (u_in > 0.0)
        return 1.0;
    else if (u_in < 0.0)
        return -1.0;
    return 0.0;
}

// 等效系數
double a = 9.9;
// double d = 291397.9;
double b = 2223.4;

// double a = 8.1662;
// double d = 121000.0;
// double b = 2002.8;

// PID gain
double KP = 1.1;
double KD = 0.1;
double KI = 2.4;
// double KP = 1322.0;
// double KD = 63.0;
// double KI = 9240.0;
int DiscreMethod_flag = 3; // 1: Back 2:Bilinear 3:Bilinear+low-pass

double alpha_1 = 0.0, alpha_2 = 0.0;
double beta_0 = 0.0, beta_1 = 0.0, beta_2 = 0.0;


int idx = 0; // Timer
double t = 0.0;
double f = 0.2; 

// Command
double pos_d_y = 0.0, pos_d_k = 0.0, pos_d_last = 0.0, pos_d_u = 0.0; 
double vel_d_y = 0.0, vel_d_k = 0.0, vel_d_last = 0.0, vel_d_u = 0.0;
double acc_d_y = 0.0, acc_d_k = 0.0, acc_d_last = 0.0, acc_d_u = 0.0;
double pos_d_y_pre = 0.0, pos_d_y_pre2 = 0.0;
double pos_d_ff = 0.0, pos_d_ff_pre = 0.0,  pos_d_ff_pre2 = 0.0;
double fc_Commandpre = 2.0; // Command Prefilter cut-of frequence
double tau = 1.0*pow(2.0*PI*fc_Commandpre,-1);

double tau3 = pow(tau,-3);
double tau2 = pow(tau,-2);
double tau1 = pow(tau,-1);

//vel low-pass
double fc_lowpass = 10;
double tau_lowpass = 1/(2*PI*fc_lowpass);
double A_1D = 1.0/(PI*fc_lowpass) + dt;
double A_2D = dt - 1.0/(PI*fc_lowpass);
double B_1D = 2;
double B_2D = -2;

void setup() {
    Serial.begin(115200); // 開啟序列埠傳輸，設定 baud rate 為 115200
    delay(3000);
    // 設定外部中斷: SWITCH_BUTTON 這個接角在電壓 FALLING 的時候，執行 'ISR_Hardware_Interrupt' 這個副程式
    attachInterrupt(digitalPinToInterrupt(SWITCH_BUTTON), ISR_Hardware_Interrupt, FALLING);

    // 定義腳位的功能
    pinMode(MOTOR_PWM_CW, OUTPUT);                       // 設定 MOTOR_PWM_CW 腳位為輸出
    pinMode(MOTOR_PWM_CCW, OUTPUT);                      // 設定 MOTOR_PWM_CCW 腳位為輸出
    analogWriteFrequency(MOTOR_PWM_CW, MOTOR_PWM_FREQ);  // 設定 PWM 的輸出頻率
    analogWriteFrequency(MOTOR_PWM_CCW, MOTOR_PWM_FREQ); // 設定 PWM 的輸出頻率
    analogWriteResolution(PWM_WRITE_RES);                // 設定所有 PWM 的解析度

    // 將馬達正轉跟負轉的 PWM 腳位設定成0，避免暴衝。
    Motor_Reset();

    // 清空 QEI 的 Encoder 內存值
    enc.reset();

    delay(1000);

    ISR_Timer.begin(ISR_Routine, ISR_TIME); // 內部中斷開始
    ISR_Timer.priority(255);
}

void loop() {
    if (ISR_Enable)
    {
        // 序列埠傳輸
        Serial.flush();
        Serial.print(System_Enable);
        Serial.print(" ");
        Serial.print(pos_d_y);
        Serial.print(" ");
        Serial.print(vel_d_y);
        Serial.print(" ");
        Serial.print(acc_d_y);
        Serial.print(" ");
        Serial.print(pos);
        Serial.print(" ");
        Serial.print(vel);
        Serial.print(" ");
        Serial.print(pos_d_ff);
        Serial.print(" ");
        Serial.print(Motor_PWM_Cmd);
        Serial.print("\n");
        ISR_Enable = 0;
    }
    // 馬達 PWM 命令更新
    // delay(10);
}

void ISR_Routine()
{
    ISR_Enable = 1;

    // 轉速參考命令
    // vel_d = Wheel.read() * 400000 / 80;

    // 參考命令

    t = (double)idx * dt;
    // pos_d_u = 15000.0 * sign(sin(2.0 * PI *f * t));
    pos_d_u = (double) Wheel_Enc.read()/80.0*60000.0;
    
    vel_d_u = 0.0;
    acc_d_u = 0.0;

    pos_d_last = vel_d_k*dt +pos_d_k;
    vel_d_last = acc_d_k*dt +vel_d_k;
    acc_d_last = (-1.0*tau3*pos_d_k -3.0*tau2*vel_d_k -3.0*tau1*acc_d_k +pos_d_u)*dt +acc_d_k;

    pos_d_y = 1.0*tau3*pos_d_k;
    vel_d_y = 1.0*tau3*vel_d_k;
    acc_d_y = 1.0*tau3*acc_d_k;

    pos_d_k = pos_d_last;
    vel_d_k = vel_d_last;
    acc_d_k = acc_d_last;

    // pos_d_y = 30000.0 * sign(sin(2.0 * PI *f * t));
    // vel_d_y = 0;
    // acc_d_y = 0;


    // 讀入馬達的 pulse 數
    enc.sync();
    pos = enc.left();

    // 速度估測 (差分)
    vel_diff = (double)(pos - pos_pre) / dt;

    // === pos low-pass === //
    vel = 1.0/A_1D*(-A_2D*vel_pre +B_1D*pos +B_2D*pos_pre);
    vel_pre = vel;

    // === vel low-pass === //
    // vel_in = (double)(pos - pos_pre) / dt;
    // vel = 1.0/A_1D*(-A_2D*vel_pre +B_1D*vel_in +B_2D*vel_in_pre);
    // vel_in_pre = vel_in;
    // vel_pre = vel;
    
    pos_pre = pos;
    // 誤差計算
    double e1 = (double) ((long int) pos_d_y - pos);

    switch (DiscreMethod_flag)
    {
    case 1:
        alpha_1 = 1.0;
        alpha_2 = 0.0;
        beta_0 = (KD +KP*dt +KI*dt*dt)/dt;
        beta_1 = (-2.0*KD -KP*dt)/dt;
        beta_2 = KD/dt;
        break;
    case 2:
        alpha_1 = 0.0;
        alpha_2 = 1.0;
        beta_0 = (2.0*KD +KP*dt +0.5*KI*dt*dt)/dt;
        beta_1 = (-4.0*KD +KI*dt*dt)/dt;
        beta_2 = (2.0*KD -KP*dt +0.5*KI*dt*dt)/dt;
        break;
    case 3:
        alpha_1 = (8.0*tau_lowpass)/(4.0*tau_lowpass+2.0*dt);
        alpha_2 = (-4.0*tau_lowpass+2.0*dt)/(4.0*tau_lowpass+2.0*dt);
        beta_0 = (4.0*KD +2.0*KP*dt +KI*dt*dt)/(4.0*tau_lowpass+2.0*dt);
        beta_1 = (-8.0*KD +2.0*KI*dt*dt)/(4.0*tau_lowpass+2.0*dt);
        beta_2 = (4.0*KD -2.0*KP*dt +KI*dt*dt)/(4*tau_lowpass+2*dt);
        break;
    }

    // pos_d_ff = acc_d_y/b +a*vel_d_y/b;


    // double u = pos_d_ff +alpha_1*u_pre +alpha_2*u_pre2 +beta_0*e1 +beta_1*e1_pre +beta_2*e1_pre2;
    double u = alpha_1*u_pre +alpha_2*u_pre2 +beta_0*e1 +beta_1*e1_pre +beta_2*e1_pre2;
    e1_pre2 = e1_pre;
    e1_pre = e1;
    pos_d_y_pre2 = pos_d_y_pre;
    pos_d_y_pre = pos_d_y;
    pos_d_ff_pre2 = pos_d_ff_pre;
    pos_d_ff_pre = pos_d_ff;
    u_pre2 = u_pre;
    u_pre = u;

    // 馬達命令
    if (System_Enable)
    {
        Motor_PWM_Cmd = (int) u;
        if (abs(Motor_PWM_Cmd) >= Motor_PWM_Max)
        {
            Motor_PWM_Cmd = sign(Motor_PWM_Cmd) * Motor_PWM_Max;
        }
    }
    else
    {
        Motor_PWM_Cmd = 0;
    }

    if (Motor_PWM_Cmd >= 0) // 代表現在要馬達正轉
    {
        analogWrite(MOTOR_PWM_CW, Motor_PWM_Cmd);
        analogWrite(MOTOR_PWM_CCW, 0);
    }
    else // 代表現在要馬達負轉
    {
        analogWrite(MOTOR_PWM_CW, 0);
        analogWrite(MOTOR_PWM_CCW, -Motor_PWM_Cmd);
    }

    idx++;
}

void ISR_Hardware_Interrupt()
{
    System_Enable = !System_Enable;

    Motor_Reset();

    // Serial.print("System Enable Flag: " + String(System_Enable) + "\n");
}

void Motor_Reset()
{
    Motor_PWM_Cmd = 0;
    analogWrite(MOTOR_PWM_CW, 0);
    analogWrite(MOTOR_PWM_CCW, 0);
}