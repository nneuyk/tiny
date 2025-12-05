#include <Arduino.h>
#include <SPI.h>
#include <Servo.h>

//IMU: ICM42688 (SPI CS_42688 --> SPI3_NSS (PA15))
//pins
#define IMU_CS     PA15    // SPI3_NSS
#define SPI_SCK    PA5    // SPI1_SCK
#define SPI_MISO   PA6    // SPI1_MISO
#define SPI_MOSI   PA7    // SPI1_MOSI
#define IMU_INT    PA0     // assumed interrupt pin

#define SERVO1 PA1
#define SERVO2 PA2

//icm42688 register addresses idk what this is chatgpt bork hai add
#define REG_PWR_MGMT0   0x4E
#define REG_GYRO_CONFIG0 0x4F
#define REG_ACCEL_CONFIG0 0x50
#define REG_ACCEL_DATA_X1 0x09 // High byte of accel X
#define REG_GYRO_DATA_X1  0x11 // High byte of gyro X

// ------------------ SPI SETTINGS ------------------------
SPISettings imuSPI(1e6, MSBFIRST, SPI_MODE0);

Servo servo1, servo2;

// ------------ Helper: Write 8-bit register --------------
void imuWrite(uint8_t reg, uint8_t val)
{
    SPI.beginTransaction(imuSPI);
    digitalWrite(IMU_CS, LOW);
    SPI.transfer(reg);
    SPI.transfer(val);
    digitalWrite(IMU_CS, HIGH);
    SPI.endTransaction();
}

// ------------ Helper: Read 8-bit register ---------------
uint8_t imuRead(uint8_t reg)
{
    SPI.beginTransaction(imuSPI);
    digitalWrite(IMU_CS, LOW);
    SPI.transfer(reg | 0x80); // Read bit
    uint8_t val = SPI.transfer(0x00);
    digitalWrite(IMU_CS, HIGH);
    SPI.endTransaction();
    return val;
}

// ------------ Read 16-bit accel value -------------------
int16_t readAccelX()
{
    uint8_t hi = imuRead(REG_ACCEL_DATA_X1);
    uint8_t lo = imuRead(REG_ACCEL_DATA_X1 + 1);
    return (int16_t)((hi << 8) | lo);
}

// ------------ Read 16-bit gyro value --------------------
int16_t readGyroX()
{
    uint8_t hi = imuRead(REG_GYRO_DATA_X1);
    uint8_t lo = imuRead(REG_GYRO_DATA_X1 + 1);
    return (int16_t)((hi << 8) | lo);
}

//imu setup
void imuInit()
{
    delay(50);

    // Enable accelerometer + gyro
    imuWrite(REG_PWR_MGMT0, 0x0F);

    // Gyro config: ±2000 dps, ODR=1kHz
    imuWrite(REG_GYRO_CONFIG0, 0x06);

    // Accel config: ±16g, ODR=1kHz
    imuWrite(REG_ACCEL_CONFIG0, 0x06);

    delay(20);
}

//setup
void setup()
{
    // SPI + pins
    pinMode(IMU_CS, OUTPUT);
    digitalWrite(IMU_CS, HIGH);

    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // IMU interrupt pin (if used)
    pinMode(IMU_INT, INPUT);

    // Serial output
    Serial.begin(115200);
    delay(200);

    // Initialize IMU without WHO_AM_I
    imuInit();

    // Attach servos
    servo1.attach(SERVO1);
    servo2.attach(SERVO2);

    Serial.println("IMU initialized. Reading data...");
}

//loop
void loop()
{
    int16_t ax = readAccelX();
    int16_t gx = readGyroX();

    Serial.print("Accel X: ");
    Serial.print(ax);
    Serial.print("   Gyro X: ");
    Serial.println(gx);

    // Example servo control
    int servoPos = map(ax, -16000, 16000, 0, 180);
    servoPos = constrain(servoPos, 0, 180);

    servo1.write(servoPos);
    servo2.write(180 - servoPos);

    delay(20);
}