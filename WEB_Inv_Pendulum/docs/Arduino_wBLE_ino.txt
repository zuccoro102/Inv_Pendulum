#include <Wire.h>
#include <MadgwickAHRS.h> //　Madgwickフィルタライブラリを取り込む (予めライブラリ"Madgwick"をインストールする)
#include <ArduinoBLE.h> // BLEのライブラリを取り込む
#include <SPI.h> // MPU-6500でSPIを使う

// サービスに割り当てるUUIDを指定
BLEService tuningService("965faa66-e492-4af1-8c42-deef6500ee8c");
// 属性に割り当てるUUIDを指定
BLEStringCharacteristic paramChar("5ba1f964-5516-4fa1-a601-688a59fe6e8f", BLEWrite | BLERead | BLENotify, 30); //書き込み,20文字

// スケール定数
#define ACC_SCALE 16384.0   // ±2g
#define GYRO_SCALE 131.0    // ±250 dps

// ジャイロオフセット格納
float gx_offset = 0, gy_offset = 0, gz_offset = 0;

//IMUセンサのデータから正確な姿勢を算出するmadgwickフィルタ
Madgwick filter;

// MPU-6500用ピン
#define CS_PIN 10

// SPI通信の設定 (1 MHz, MSBFIRST, MODE0)を定義
SPISettings spiSettings(1000000, MSBFIRST, SPI_MODE0);

// DRV8835用モータ制御関係
const int PHASE_A = 7; // モータAの正転・反転  -- DriverのPIN = BIN1, ArduinoのPIN = 7
const int ENABLE_A = 6; // モータAのPWM  -- DriverのPIN = BIN2, ArduinoのPIN = 6
const int PHASE_B = 4; // モータBの正転・反転    -- DriverのPIN = AIN1, ArduinoのPIN = 4
const int ENABLE_B = 5; // モータBのPWM  -- DriverのPIN = AIN2, ArduinoのPIN = 5
const int MODE_PWM = 8; // PWMを使うモード  ArduinoのPIN = 8

#define V_MIN 20 // モータ駆動PWM(0~255)の最低値。これより低いと回らない。
#define V_MAX 255 // モータ駆動PWM(0~255)の最高値。

// Roll, Pitch, Yawの初期値
float Roll = 0.;
float Pitch = 0.;
float Yaw = 0.;
float prevPitch = 0.;
float pos = 0.; // 位置
float vel = 0.; // 速度
float prevPos = 0.;

// pid制御パラメータの初期
float Kp = 3000.; // Pゲイン
float Ki = 5.0; // Iゲイン
float Kd = 10.0; // Dゲイン
float Ks = 50.0; // Pゲインの2乗
float Kvel = 0.0; // 速度FBゲイン0.1
float Kpos = 0.; // 位置FBゲイン0.5
float Kpfilt = 0.05; // Pのfilter係数
float Kdfilt = 0.95; // Dのfilter係数
float Kposdec = 0.999; // posの減衰係数
float Kpwma = 1.05; // モータAのPWMの係数、基準は1.0 右側
float Kpwmb = 0.95; // モータBのPWMの係数、基準は1.0　左側
float Kt = 0.7; // 電池の消耗を考慮する係数、最初は~0.7、最後は~1.0、pwmの係数

float target = -6.6; // 目標値　大きくすると前にいく

// 制御用
float dt; // 処理時間
float P, I = 0, D;
float Power;  // モータの出力(PID計算結果)
int pwm, pwma, pwmb;  // pwmデューティー比 0~255
int stoptheta = 20; // 倒れすぎたらモータを止める角度
int Ncount = 0; // カウント
int Ncmax = 3000; // カウントの最大値

// Dフィルタ用
float D_filtered = 0;

// Pフィルタ用
float P_filtered = 0;

//タイミング
unsigned long prevMicros = 0;
const int LOOP_US = 5000; // 200 Hz

unsigned long lastControl = 0;
unsigned long lastBle = 0;

// 操作用
float target_offset = 0.;
float turn_offset = 0.;
float target_cmd = 0.;
float turn_cmd = 0.;

// 状態判定
bool isSafe = false;

// MPU-6500から読み込む
void readMPU6500(int16_t &ax, int16_t &ay, int16_t &az, int16_t &gx, int16_t &gy, int16_t &gz) {
  SPI.beginTransaction(spiSettings); // SPI通信の開始を明示
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x3B | 0x80); // 読み出し開始

  ax = (SPI.transfer(0) << 8) | SPI.transfer(0);
  ay = (SPI.transfer(0) << 8) | SPI.transfer(0);
  az = (SPI.transfer(0) << 8) | SPI.transfer(0);
  SPI.transfer(0); SPI.transfer(0); // temperature skip
  gx = (SPI.transfer(0) << 8) | SPI.transfer(0);
  gy = (SPI.transfer(0) << 8) | SPI.transfer(0);
  gz = (SPI.transfer(0) << 8) | SPI.transfer(0);

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction(); // SPI通信の終了
}

// MPU-6500 ジャイロオフセット補正 1秒間
void calibrateGyro() {
  int samples = 200;
  long gx_sum = 0, gy_sum = 0, gz_sum = 0;
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  for (int i = 0; i < samples; i++) {
    // RAW値取得
    readMPU6500(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);
    gx_sum += gx_raw;
    gy_sum += gy_raw;
    gz_sum += gz_raw;
    delay(5);
  }
  
  // オフセット値
  gx_offset = gx_sum / (float)samples;
  gy_offset = gy_sum / (float)samples;
  gz_offset = gz_sum / (float)samples;
}

// MPU-6500用
void writeRegister(uint8_t reg, uint8_t data){
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg);
  SPI.transfer(data);
  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}

// MPU-6500用
uint8_t readRegister(uint8_t reg){
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(reg | 0x80);
  uint8_t val = SPI.transfer(0x00);
  digitalWrite(CS_PIN, HIGH);
  return val;
}

// MPU-6500用
void initMPU6500(){
  writeRegister(0x6B, 0x00); // sleep解除
  delay(100);
}

void controlLoop(){
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;
  
  // 時間差計算（秒）
  unsigned long now = micros();
  float dt = (now - prevMicros) / 1000000.0; // 秒の単位にする
  dt = constrain(dt, 0.002, 0.01); // 転倒防止_20260326
  prevMicros = now;

  // RAW値取得
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  readMPU6500(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  gx_raw -= gx_offset;
  gy_raw -= gy_offset;
  gz_raw -= gz_offset;

  // 物理単位換算　加速度値を分解能で割って加速度(g)に変換する
  float ax = ax_raw / ACC_SCALE;
  float ay = ay_raw / ACC_SCALE;
  float az = az_raw / ACC_SCALE;

  // 角速度値を分解脳で割って角速度(degree/s)に変換し、さらにrad/sに変換する
  // float gx = gx_raw / GYRO_SCALE * PI / 180.0; // rad/s
  // float gy = gy_raw / GYRO_SCALE * PI / 180.0;
  // float gz = gz_raw / GYRO_SCALE * PI / 180.0;

  float gx = gx_raw / GYRO_SCALE; // deg/s
  float gy = gy_raw / GYRO_SCALE;
  float gz = gz_raw / GYRO_SCALE;

  // Madgwickフィルタ更新
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  
  // Roll/Pitch/Yaw取得（°単位）
  // Roll  = filter.getRoll(); // Rollの推定角度 degree
  Pitch = filter.getPitch(); // PITCHの推定角度 degree
  // Yaw   = filter.getYaw(); // YAWの推定角度 degree

  /*
  // 確認用表示
  Serial.print("ACC (g): ");
  Serial.print(ax, 2); Serial.print(" ");
  Serial.print(ay, 2); Serial.print(" ");
  Serial.print(az, 2);

  Serial.print( "ROLL :" ); Serial.print( Roll ); Serial.print( "," );
  Serial.print( "PITCH:" ); Serial.print( Pitch ); Serial.print( "," );
  Serial.print( "YAW  :" ); Serial.print( Yaw );
  Serial.print("\n");
  */
  
  // PID制御
  // 目標から現在の角度を引いて偏差を求める
  target_offset = target_offset + 0.001 * (target_cmd - target_offset); // ゆっくり変化させる
  float P_raw = ((target + target_offset) - Pitch)/90.0; // -90～90を取るので90で割って-1.0～1.0にする

  // ローパスフィルタ
  // P_filtered = 0.01 * P_filtered + 0.99 * P_raw;
  P_filtered = Kpfilt * P_filtered + (1. - Kpfilt) * P_raw;
  P = P_filtered;

  // Pが小さい時Iを減衰させる
  if (abs(P) < 0.01){
    I *= 0.99;
  }
  
  // 積分 (制限付き)
  I += P * dt; // 偏差を積分する
  I = constrain(I, -100, 100); // 制限

  // 微分 (pitchベース)
  float pitchRate = (Pitch - prevPitch) / dt;
  prevPitch = Pitch; // 偏差を記録

  float D_raw = -pitchRate;

  // ローパスフィルタ
  D_filtered = Kdfilt * D_filtered + (1. - Kdfilt) * D_raw;
  D = D_filtered;

  /*
  // 確認用表示
  Serial.print("Pitch =\t"); Serial.print(Pitch); Serial.print(", \t");
  Serial.print("Kp*P =\t"); Serial.print(Kp*P,4); Serial.print(", \t");
  Serial.print("Ki*I =\t"); Serial.print(Ki*I,4); Serial.print(", \t");
  Serial.print("Kd*D =\t"); Serial.print(Kd*D,4); Serial.print(", \n");
  */

  // 角度を検知してモータを動作させる（倒立振り子の主動作）
  // 出力を計算する
  Power = Kp * P * (1. + Ks * abs(P)) + Ki * I + Kd * D;

  // 電池の消耗を考慮
  Power = Kt * Power;

  //最初しばらくは位置、速度は計算しない
  if (Ncount < Ncmax){
    Ncount += 1;
    pos = 0.;
    vel = 0.;
    prevPos = 0.;
  }

  // vel = Power; // 速度
  Power = Power + Kpos * pos + Kvel * vel; // 位置のFB
  // Power = Kp * P + Ki * I + Kd * D;

  float Powerlim = constrain(Power, -255., 255.); // 255.に制限されるので
  pos = Kposdec * pos + Powerlim * dt; // 位置
  vel = (pos - prevPos) / dt;
  prevPos = pos;

  // もしtarget_cmdがある値を持っているならpos、prevPosをゼロにする
  // if (abs(target_offset > 1.0)){
  //  pos = 0.;
  //   prevPos = 0.;
  // }

  // モータのPWM出力
  pwm = constrain(abs((int)Power), V_MIN, V_MAX); // 255に制限する
  
  // 左右のモータの違いを考慮
  turn_offset = turn_offset + 0.0008 * (turn_cmd - turn_offset); // ゆっくり変化させる
  pwma = (int)(Kpwma * pwm - turn_offset);
  pwmb = (int)(Kpwmb * pwm + turn_offset);

  // 制限
  pwma = constrain(pwma, 0, 255);
  pwmb = constrain(pwmb, 0, 255);

  // フェイルセーフ
  isSafe = false;

  if (Pitch < -stoptheta + target || Pitch > stoptheta + target){
    // 場外フラグ
    isSafe = true;

    // DRV8835用倒れすぎたら停止
    analogWrite( ENABLE_A, 0); // モータAを停止
    analogWrite( ENABLE_B, 0); // モータBを停止

    P = 0;
    I = 0;
    D = 0;
    pos = 0.;
    vel = 0.;
    Ncount = 0;

    D_filtered = 0.;
    P_filtered = 0.;
    prevPitch = 0.;
    prevPos = 0.;

    target_offset = 0.;
    turn_offset = 0.;
    target_cmd = 0.;
    turn_cmd = 0.;

    return;
  }
  
  if (Power > 0){
    // DRV8835用正転
    digitalWrite( PHASE_A, LOW ); // モータAを正転にする
    digitalWrite( PHASE_B, LOW ); // モータBを正転にする
  }else{
    digitalWrite( PHASE_A, HIGH ); //モータAを反転にする
    digitalWrite( PHASE_B, HIGH ); // モータBを反転にする  
  }
  
  analogWrite( ENABLE_A, pwma ); // モータAをpwmaで動かす
  analogWrite( ENABLE_B, pwmb ); // モータBをpwmbで動かす

  // 減衰
  target_offset *= 0.9996;
  turn_offset *= 0.9996;
  target_cmd *= 0.9996;
  turn_cmd *= 0.9996;
  
}

void bleLoop(){
  BLE.poll();

  if (paramChar.written()){ // 属性が書き換えられたかどうかを確認する
    String cmd = paramChar.value();

    // パラメータ変更(安全時のみ)
    if (isSafe){
      if (cmd.startsWith("Kp=")) Kp = cmd.substring(3).toFloat();
      if (cmd.startsWith("Ki=")) Ki = cmd.substring(3).toFloat();
      if (cmd.startsWith("Kd=")) Kd = cmd.substring(3).toFloat();
      if (cmd.startsWith("Ks=")) Ks = cmd.substring(3).toFloat();
      if (cmd.startsWith("Kt=")) Kt = cmd.substring(3).toFloat();

      // Serial.print("Kp = ");Serial.print(Kp);Serial.print("\n");
      // Serial.print("Ki = ");Serial.print(Ki);Serial.print("\n");
      // Serial.print("Kd = ");Serial.print(Kd);Serial.print("\n");
      // Serial.print("Ks = ");Serial.print(Ks);Serial.print("\n");
      // Serial.print("Kt = ");Serial.print(Kt);Serial.print("\n");

      // 3つの値をカンマ区切りなどで1つの文字列にする
      // 例: "Kp:1.0, Ki:2.0, Kd:0.5"
      String statusMsg1 = "Kp:" + String(Kp, 2) + ", Ki:" + String(Ki, 2) + ", Kd:" + String(Kd, 2);
      String statusMsg2 = "Ks:" + String(Ks, 2) + ", Kt:" + String(Kt, 2);
      
      // まとめた文字列をブラウザに通知
      paramChar.writeValue(statusMsg1);
      paramChar.writeValue(statusMsg2);

      return;

      // Serial.print("Received and Notified: "); Serial.println(cmd);
    }
    
    // 操作コマンド
    if (cmd == "f") target_cmd += 1.4; // 最大で約1/3になる
    if (cmd == "b") target_cmd -= 1.4;
    if (cmd == "r"){
      turn_cmd += 60.;
      target_cmd += 0.8; // 少し前進する
    }
    if (cmd == "l"){
      turn_cmd -= 60.;
      target_cmd += 0.8; // 少し前進する
    }

    // 文字列をブラウザに通知
    String statusMsg = "cmd :" + cmd + " " + String(target_cmd, 2) + " " + String(turn_cmd, 2);
    paramChar.writeValue(statusMsg);
  }

}

void setup() {
  // シリアル出力On 1秒間に115,200ビットの通信速度(ボーレート)でシリアル通信の準備をする
  Serial.begin(115200);
  delay(2000);
  
  // SPI開始
  SPI.begin();
  pinMode(CS_PIN, OUTPUT);
  digitalWrite(CS_PIN, HIGH);

  // MPU-6500 初期化 sleep解除
  initMPU6500();

  // ジャイロオフセット補正
  Serial.println("Calibrating gyro... Keep the board still");
  calibrateGyro();
  Serial.println("Calibration done!");

  // Madgwickフィルタ初期化
  filter.begin(200); // 200 Hz

  // DRV8835用モータ制御関係の端子を出力端子にする
  pinMode( PHASE_A, OUTPUT );
  pinMode( ENABLE_A, OUTPUT );
  pinMode( PHASE_B, OUTPUT );
  pinMode( ENABLE_B, OUTPUT );
  pinMode( MODE_PWM, OUTPUT );
  digitalWrite(MODE_PWM, HIGH); //　モータスリープ解除(常時ON)　PWMが使えるモードにする

  // DRV8835用モータを停止する(Enableを0にする)
  digitalWrite(PHASE_A, LOW);
  analogWrite(ENABLE_A, 0);
  digitalWrite(PHASE_B, LOW);
  analogWrite(ENABLE_B, 0);

  // タイム管理
  prevMicros = micros();

  // BLE初期化
  BLE.begin(); // BLEを使用できるようにする
  BLE.setLocalName("InvertedPendulum"); // サービスの名前を指定する
  BLE.setAdvertisedService(tuningService); // アドバタイズするサービスをセットする
  tuningService.addCharacteristic(paramChar);// サービスに属性を追加する
  BLE.addService(tuningService);// 指定したサービスを利用できるように登録処理する
  BLE.setAdvertisedServiceUuid(tuningService.uuid()); // アドバタイズデータにService UUIDを含める
  BLE.advertise(); // アドバタイズの送信を開始する

  BLE.poll();
}

void loop() {
  unsigned long now0 = micros();

  // 高速制御ループ (200Hz固定)
  if (now0 - lastControl >= 5000){
    lastControl += 5000;
    controlLoop();
  }
  
  // 低速BLE処理
  if (millis() - lastBle >= 100){
    lastBle = millis();
    bleLoop();
  }
 
}
