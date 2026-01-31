#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <EEPROM.h>

/* ---------------- AYARLAR ---------------- */

#define BNO055_SAMPLERATE_DELAY_MS (25)
#define CS_PIN 10    // SPI Chip Select

/* I2C address: 0x28 or 0x29 */
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

/* SPI buffer (3 float = 12 byte) */
uint8_t spiBuffer[12];

/* ---------------- YARDIMCI FONKSIYONLAR ---------------- */

void displaySensorDetails(void)
{
    sensor_t sensor;
    bno.getSensor(&sensor);
    Serial.println("------------------------------------");
    Serial.print("Sensor:       "); Serial.println(sensor.name);
    Serial.print("Driver Ver:   "); Serial.println(sensor.version);
    Serial.print("Unique ID:    "); Serial.println(sensor.sensor_id);
    Serial.print("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
    Serial.print("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
    Serial.print("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
    Serial.println("------------------------------------");
    Serial.println("");
    delay(500);
}

void displaySensorStatus(void)
{
    uint8_t system_status, self_test_results, system_error;
    bno.getSystemStatus(&system_status, &self_test_results, &system_error);

    Serial.println("");
    Serial.print("System Status: 0x");
    Serial.println(system_status, HEX);
    Serial.print("Self Test:     0x");
    Serial.println(self_test_results, HEX);
    Serial.print("System Error:  0x");
    Serial.println(system_error, HEX);
    Serial.println("");
    delay(500);
}

void displayCalStatus(void)
{
    uint8_t system, gyro, accel, mag;
    bno.getCalibration(&system, &gyro, &accel, &mag);

    Serial.print("\t");
    if (!system) Serial.print("! ");

    Serial.print("Sys:"); Serial.print(system);
    Serial.print(" G:");   Serial.print(gyro);
    Serial.print(" A:");   Serial.print(accel);
    Serial.print(" M:");   Serial.print(mag);
}

void displaySensorOffsets(const adafruit_bno055_offsets_t &calibData)
{
    Serial.print("Accelerometer: ");
    Serial.print(calibData.accel_offset_x); Serial.print(" ");
    Serial.print(calibData.accel_offset_y); Serial.print(" ");
    Serial.print(calibData.accel_offset_z); Serial.print(" ");

    Serial.print("\nGyro: ");
    Serial.print(calibData.gyro_offset_x); Serial.print(" ");
    Serial.print(calibData.gyro_offset_y); Serial.print(" ");
    Serial.print(calibData.gyro_offset_z); Serial.print(" ");

    Serial.print("\nMag: ");
    Serial.print(calibData.mag_offset_x); Serial.print(" ");
    Serial.print(calibData.mag_offset_y); Serial.print(" ");
    Serial.print(calibData.mag_offset_z); Serial.print(" ");

    Serial.print("\nAccel Radius: ");
    Serial.print(calibData.accel_radius);

    Serial.print("\nMag Radius: ");
    Serial.print(calibData.mag_radius);
}

/* ---------------- SETUP ---------------- */

void setup(void)
{
    Serial.begin(115200);
    delay(1000);
    Serial.println("BNO055 Orientation + SPI TX");

    /* SPI init */
    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    SPI.begin();

    /* BNO055 init */
    if (!bno.begin())
    {
        Serial.println("No BNO055 detected!");
        while (1);
    }

    int eeAddress = 0;
    long bnoID;
    bool foundCalib = false;

    EEPROM.get(eeAddress, bnoID);

    adafruit_bno055_offsets_t calibrationData;
    sensor_t sensor;
    bno.getSensor(&sensor);

    if (bnoID == sensor.sensor_id)
    {
        Serial.println("Found calibration in EEPROM");
        eeAddress += sizeof(long);
        EEPROM.get(eeAddress, calibrationData);
        bno.setSensorOffsets(calibrationData);
        foundCalib = true;
    }

    bno.setExtCrystalUse(true);

    sensors_event_t event;
    if (!foundCalib)
    {
        Serial.println("Please calibrate sensor...");
        while (!bno.isFullyCalibrated())
        {
            bno.getEvent(&event);
            displayCalStatus();
            Serial.println("");
            delay(BNO055_SAMPLERATE_DELAY_MS);
        }

        adafruit_bno055_offsets_t newCalib;
        bno.getSensorOffsets(newCalib);

        eeAddress = 0;
        bnoID = sensor.sensor_id;
        EEPROM.put(eeAddress, bnoID);
        eeAddress += sizeof(long);
        EEPROM.put(eeAddress, newCalib);
    }

    displaySensorDetails();
    displaySensorStatus();
}

/* ---------------- LOOP ---------------- */

void loop(void)
{
    sensors_event_t event;
    bno.getEvent(&event);

    float x = event.orientation.x;
    float y = event.orientation.y;
    float z = event.orientation.z;

    /* Serial debug */
    Serial.print("X: "); Serial.print(x, 4);
    Serial.print("\tY: "); Serial.print(y, 4);
    Serial.print("\tZ: "); Serial.print(z, 4);
    displayCalStatus();
    Serial.println("");

    /* ---------- SPI GONDERIM ---------- */

    memcpy(&spiBuffer[0], &x, 4);
    memcpy(&spiBuffer[4], &y, 4);
    memcpy(&spiBuffer[8], &z, 4);

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(CS_PIN, LOW);

    for (int i = 0; i < 12; i++)
    {
        SPI.transfer(spiBuffer[i]);
    }

    digitalWrite(CS_PIN, HIGH);
    SPI.endTransaction();

    delay(BNO055_SAMPLERATE_DELAY_MS);
}
