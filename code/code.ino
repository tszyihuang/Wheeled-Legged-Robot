/**
 * ESP32-S3 平衡轮腿机器人固件
 * 硬件: ESP32S3, SimpleFOC Mini x2, 2208电机 x2, PCA9685, MPU6050
 */

#include <SimpleFOC.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <MPU6050_light.h>

// ================= 1. 定义引脚与对象 =================

// --- I2C 定义 ---
#define I2C0_SDA 1
#define I2C0_SCL 2
#define I2C1_SDA 15
#define I2C1_SCL 16

// --- MPU6050 ---
// 连接在 I2C0
MPU6050 mpu(Wire);
#define MPU_ADO_PIN 6 // 你的 ADO 接了 Pin 6
#define MPU_INT_PIN 7

// --- 舵机 (PCA9685) ---
// 连接在 I2C0
Adafruit_PWMServoDriver servos = Adafruit_PWMServoDriver(0x40);

// --- 电机与驱动 (3PWM模式) ---
// 假设 SimpleFOC Mini 使用 3PWM 控制
// 电机 1 (左?)
BLDCMotor motor1 = BLDCMotor(7); // 2208电机通常是7对极，请确认
BLDCDriver3PWM driver1 = BLDCDriver3PWM(40, 41, 42); 
// 编码器 1 (AS5600等 I2C磁编码器，连接在 I2C0)
MagneticSensorI2C sensor1 = MagneticSensorI2C(AS5600_I2C);

// 电机 2 (右?)
BLDCMotor motor2 = BLDCMotor(7); 
BLDCDriver3PWM driver2 = BLDCDriver3PWM(39, 38, 37);
// 编码器 2 (连接在 I2C1)
MagneticSensorI2C sensor2 = MagneticSensorI2C(AS5600_I2C);

// ================= 2. 控制参数 =================

// --- 直立环 (Stabilization) ---
// 目标：保持 Pitch 角度为 0 (或由速度环给出的偏移量)
// 你测试过的参数: P=10, D=0.2
PIDController pid_stb(6, 0, 0.02, 10000, 20); // P, I, D, Ramp, Limit

// --- 速度环 (Velocity) ---
// 目标：保持车轮转速为 0
PIDController pid_vel(0.05, 0.005, 0, 1000, 10); // 需要调试，先给小值
LowPassFilter lpf_vel(0.1); // 速度低通滤波

// --- 变量 ---
float target_pitch = 0; // 目标角度
float pitch_offset = 1.6; // 机械平衡零点偏移(如果车重心不正，调整此值)
float target_speed = 0; // 目标速度 (0 = 静止)
float steering = 0;     // 转向分量

// --- 调参命令 ---
Commander command = Commander(Serial);
void doPIDStb(char* cmd) { command.pid(&pid_stb, cmd); }
void doPIDVel(char* cmd) { command.pid(&pid_vel, cmd); }
void doTarget(char* cmd) { command.scalar(&target_speed, cmd); }
void doOffset(char* cmd) { command.scalar(&pitch_offset, cmd); }
// ================= 3. 辅助函数 =================

// 将 0-180度 映射到 120-480 脉冲宽
void setServoAngle(uint8_t n, double angle) {
  if (angle < 0) angle = 0;
  if (angle > 180) angle = 180;
  // 映射: 0->120, 180->480
  int pulse = map(angle, 0, 180, 120, 480);
  servos.setPWM(n, 0, pulse);
}

// ================= 4. Setup 初始化 =================

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("System Init Start...");

  // --- I2C 总线初始化 ---
  // Wire (I2C0): MPU6050, Servo, Sensor1
  Wire.setPins(I2C0_SDA, I2C0_SCL);
  Wire.begin();
  
  // Wire1 (I2C1): Sensor2
  Wire1.setPins(I2C1_SDA, I2C1_SCL);
  Wire1.begin();

  // --- 1. MPU6050 初始化 ---
  // 设置 ADO 引脚为 LOW，确保地址是 0x68 (默认)
  pinMode(MPU_ADO_PIN, OUTPUT);
  digitalWrite(MPU_ADO_PIN, LOW); 
  
  byte status = mpu.begin();
  Serial.print("MPU6050 status: "); Serial.println(status);
  Serial.println("Calibrating MPU6050... Keep robot still!");
  mpu.calcOffsets(); // 启动时校准，保持静止！
  Serial.println("MPU6050 Ready.");

  // --- 2. 舵机初始化 ---
  servos.begin();
  servos.setPWMFreq(50); // 模拟舵机频率 50Hz
  delay(10);
  // 锁定舵机角度
  // 0路:90度, 1路:85度, 2路:95度, 3路:90度
  setServoAngle(0, 90);
  setServoAngle(1, 100);
  setServoAngle(2, 110);
  setServoAngle(3, 100);
  Serial.println("Servos Locked.");

  // --- 3. SimpleFOC 电机初始化 ---
  
  // 传感器 1
  sensor1.init(&Wire);
  motor1.linkSensor(&sensor1);
  
  // 驱动 1
  driver1.voltage_power_supply = 12; // 假设12V供电，请根据实际修改
  driver1.init();
  motor1.linkDriver(&driver1);

  // 传感器 2 (注意这里用 Wire1)
  sensor2.init(&Wire1);
  motor2.linkSensor(&sensor2);

  // 驱动 2
  driver2.voltage_power_supply = 12; 
  driver2.init();
  motor2.linkDriver(&driver2);

  // 电机控制模式：力矩(电压)控制
  motor1.controller = MotionControlType::torque;
  motor2.controller = MotionControlType::torque;

  // 启用监控 (调试用，正式跑可以注释掉)
  // motor1.useMonitoring(Serial);

  // 初始化电机
  motor1.init();
  motor2.init();

  // 校准磁编码器零点 (FOC init)
  Serial.println("Aligning Motor 1...");
  motor1.initFOC(); 
  Serial.println("Aligning Motor 2...");
  motor2.initFOC();

  // --- 4. 串口命令设置 ---
  command.add('S', doPIDStb, "Stabilize PID"); // 输入 SA10 设置 P=10
  command.add('V', doPIDVel, "Velocity PID");  // 输入 VP0.1 设置 P=0.1
  command.add('T', doTarget, "Target Speed");  // 输入 T0 停止
  command.add('O', doOffset, "Motion Zero Offset");
  Serial.println("Ready for Balancing!");
}

// ================= 5. Main Loop =================

long loop_count = 0;
float voltage_control = 0;

void loop() {
  // 1. FOC 核心循环 (必须尽可能快地运行)
  motor1.loopFOC();
  motor2.loopFOC();

  // 2. 控制逻辑 (由于 MPU 读取较慢，我们降频运行控制环，例如每 5ms 一次)
  // 或者直接运行，但在 loopFOC 之间插入
  static unsigned long last_process_time = 0;
  if (millis() - last_process_time >= 5) { // 200Hz 控制频率
    last_process_time = millis();

    // A. 读取数据
    mpu.update(); 
    float pitch = mpu.getAngleY(); // 假设 Y 轴是你的 Pitch 轴
    float gyro_rate = mpu.getGyroY(); // 角速度

    // 获取电机速度 (弧度/秒)
    float v1 = motor1.shaft_velocity;
    float v2 = motor2.shaft_velocity;
    float avg_speed = (v1 + v2) / 2.0; // 平均速度
    
    // 速度滤波
    float current_speed = lpf_vel(avg_speed);

    // B. 速度环 (外环)
    // 输入：速度误差 (目标速度 - 当前速度)
    // 输出：目标倾角 (要想加速，就得前倾)
    // 这里的 target_speed 通常为 0 (原地站立)
    float target_angle_raw = pid_vel(target_speed - current_speed);
    
    // 限制目标倾角，防止速度环要求过大的倾斜导致倒地 (例如限制在 +/- 15度)
    target_angle_raw = constrain(target_angle_raw, -15, 15);

    // C. 直立环 (内环)
    // 输入：角度误差 (目标角度 - (当前角度 - 机械零点))
    // 注意：如果向前倒 (Pitch > 0)，需要正向转矩冲过去
    float balance_control = pid_stb((target_angle_raw + pitch_offset) - pitch);

    // 如果倒下太厉害 (超过40度)，关闭电机保护
    if (abs(pitch) > 40) {
      voltage_control = 0;
      pid_stb.reset(); // 重置积分项
      pid_vel.reset();
    } else {
      voltage_control = balance_control;
    }
  }

  // 3. 执行电机动作
  // 根据电机安装方向，可能其中一个需要取反
  // 假设 X 正方向前进，如果 pitch>0 (前倾)，voltage 应该 >0 (车轮向前转)
  motor1.move(voltage_control); 
  motor2.move(-voltage_control); // 如果电机2装反了，这里改为 -voltage_control

  // 4. 串口命令监听
  command.run();
}