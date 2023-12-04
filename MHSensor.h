#ifndef MHSENSOR_H_
#define MHSENSOR_H_

/**
 * @file MHsensor.cpp
 * @author kevin chaux (kevin.chaux3@gmail.com)
 * @brief
 * @version 0.1
 * @date 2023-04-06
 *
 * @copyright Copyright (c) 2023
 *  More explained in: https://www.winsen-sensor.com/d/files/MH-440D.pdf
 *                     https://www.winsen-sensor.com/d/files/mh-440d-ndir-infrared-ch4-gas-sensor-v2_8.pdf
 */

#include <Arduino.h>

class MHSensor
{
private:
    byte _pin;
    byte _voltResolution = 3.3;  // Esp32 use 3.3
    byte _adcBitResolution = 12; // Esp32 use 12 Bits - 4095
    bool _debug = false;
    float _adc, _sensorVolt, _maxMeasuring = 50000.0f, _minMeasuring = 0.0f;

public:
    static constexpr int CH4_DATA_LENGTH = 9;
    static constexpr float MIN_OUTPUT_SIGNAL = 0.400f;
    static constexpr float MAX_OUTPUT_SIGNAL = 2.0f;

    MHSensor(
        int pin,
        int adcBitResolution = 12,
        float voltageResolution = 3.3,
        float minMeasuring = 0.0f,
        float maxMeasuring = 50000.0f,
        bool debug = false);
    void init();

    uint16_t getAnalogRead();
    float getVoltage();
    float readFromAnalog();
    float readFromUART(HardwareSerial *serial_port);
    float mapOutputVoltage(float, float, float, float, float);
    boolean calibrateToZeroPoint(HardwareSerial *serial_port);
};

#endif
