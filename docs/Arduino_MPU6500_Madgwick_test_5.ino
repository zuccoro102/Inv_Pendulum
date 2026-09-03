#include <Wire.h>
#include <MadgwickAHRS.h> //　Madgwickフィルタライブラリを取り込む (予めライブラリ"Madgwick"をインストールする)
#include <SPI.h> // MPU-6500でSPIを使う

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

// Roll, Pitch, Yawの初期値
float Roll = 0.;
float Pitch = 0.;
float Yaw = 0.;
float ax = 0;
float ay = 0;
float az = 0;
float gxx = 0;
float gyy = 0;
float gzz = 0;

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

/*
void readMPU6500(int16_t &ax_r, int16_t &ay_r, int16_t &az_r, int16_t &gx_r, int16_t &gy_r, int16_t &gz_r) {
  SPI.beginTransaction(spiSettings);
  digitalWrite(CS_PIN, LOW);
  SPI.transfer(0x3B | 0x80); // 読み出し開始

  // 加速度（こちらも念のため結合順を揃えます）
  uint8_t ax_h = SPI.transfer(0); uint8_t ax_l = SPI.transfer(0);
  ax_r = (ax_h << 8) | ax_l;
  
  uint8_t ay_h = SPI.transfer(0); uint8_t ay_l = SPI.transfer(0);
  ay_r = (ay_h << 8) | ay_l;
  
  uint8_t az_h = SPI.transfer(0); uint8_t az_l = SPI.transfer(0);
  az_r = (az_h << 8) | az_l;

  // 温度をスキップ
  SPI.transfer(0); SPI.transfer(0); 

  // ★ジャイロの結合順を逆にする（通信相性対策）
  uint8_t gx_1 = SPI.transfer(0); uint8_t gx_2 = SPI.transfer(0);
  gx_r = (gx_2 << 8) | gx_1; // 前後を入れ替えて結合してみる
  
  uint8_t gy_1 = SPI.transfer(0); uint8_t gy_2 = SPI.transfer(0);
  gy_r = (gy_2 << 8) | gy_1;
  
  uint8_t gz_1 = SPI.transfer(0); uint8_t gz_2 = SPI.transfer(0);
  gz_r = (gz_2 << 8) | gz_1;

  digitalWrite(CS_PIN, HIGH);
  SPI.endTransaction();
}
*/

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
// uint8_t readRegister(uint8_t reg){
//   digitalWrite(CS_PIN, LOW);
//   SPI.transfer(reg | 0x80);
//   uint8_t val = SPI.transfer(0x00);
//   digitalWrite(CS_PIN, HIGH);
//   return val;
// }

// MPU-6500用
void initMPU6500(){
  writeRegister(0x6B, 0x00); // sleep解除
  delay(100);
}

void controlLoop(){
  int16_t ax_raw, ay_raw, az_raw;
  int16_t gx_raw, gy_raw, gz_raw;

  // RAW値取得
  // SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
  readMPU6500(ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw);

  gx_raw -= gx_offset;
  gy_raw -= gy_offset;
  gz_raw -= gz_offset;

  // 物理単位換算　加速度値を分解能で割って加速度(g)に変換する
  ax = ax_raw / ACC_SCALE;
  ay = ay_raw / ACC_SCALE;
  az = az_raw / ACC_SCALE;

  // 角速度値を分解脳で割って角速度(degree/s)に変換し、さらにrad/sに変換する
  // float gx = gx_raw / GYRO_SCALE * PI / 180.0; // rad/s
  // float gy = gy_raw / GYRO_SCALE * PI / 180.0;
  // float gz = gz_raw / GYRO_SCALE * PI / 180.0;

  float gx = gx_raw / GYRO_SCALE;
  float gy = gy_raw / GYRO_SCALE;
  float gz = gz_raw / GYRO_SCALE;

  gxx = gx;
  gyy = gy;
  gzz = gz;

  // Madgwickフィルタ更新
  filter.updateIMU(gx, gy, gz, ax, ay, az);
  
  // Roll/Pitch/Yaw取得（°単位）
  Roll  = filter.getRoll(); // Rollの推定角度 degree
  Pitch = filter.getPitch(); // PITCHの推定角度 degree
  Yaw   = filter.getYaw(); // YAWの推定角度 degree

  // 確認用表示
  // Serial.print("ACC (g): ");
  // Serial.print(ax, 2); Serial.print(" ");
  // Serial.print(ay, 2); Serial.print(" ");
  // Serial.print(az, 2);

  // Serial.print( "ROLL :" ); Serial.print( Roll ); Serial.print( "," );
  // Serial.print( "PITCH:" ); Serial.print( Pitch ); Serial.print( "," );
  // Serial.print( "YAW  :" ); Serial.print( Yaw );
  // Serial.print("\n");
 
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

}

void loop() {
  unsigned long now = millis();
  static unsigned long lastLoopTime = micros();
  static unsigned long lastPrintTime = 0;

  // フィルタの更新(200 Ha = 5000μ秒周期)
  unsigned long nowMicros = micros();
  if (nowMicros - lastLoopTime >=5000){
    lastLoopTime = nowMicros;
    controlLoop();
  }
  
  //画面への表示(1秒 = 1000ミリ秒間隔で間引く)
  if (now - lastPrintTime >= 20){
    lastPrintTime = now;

    // 確認用表示
    /*
    Serial.print("ACC (g): ");
    Serial.print(ax, 2); Serial.print(" ");
    Serial.print(ay, 2); Serial.print(" ");
    Serial.print(az, 2); Serial.print("  |  ");

    Serial.print( "ROLL :" ); Serial.print( Roll ); Serial.print( "," );
    Serial.print( "PITCH:" ); Serial.print( Pitch ); Serial.print( "," );
    Serial.print( "YAW  :" ); Serial.print( Yaw ); Serial.print("  |  ");
  
    Serial.print(gxx, 2); Serial.print(" ");
    Serial.print(gyy, 2); Serial.print(" ");
    Serial.println(gzz, 2);
    */

    Serial.print("ROLL:");
    Serial.print(Roll, 2);
    Serial.print(",");

    Serial.print("PITCH:");
    Serial.println(Pitch, 2);

  }
  
}
