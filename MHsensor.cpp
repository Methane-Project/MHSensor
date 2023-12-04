#include "MHSensor.h"

MHSensor::MHSensor(int pin,
                   int adcBitResolution,
                   float voltageResolution,
                   float minMeasuring,
                   float maxMeasuring,
                   bool debug)
{
    this->_pin = pin;
    this->_voltResolution = voltageResolution;
    this->_adcBitResolution = adcBitResolution;
    this->_minMeasuring = minMeasuring;
    this->_maxMeasuring = maxMeasuring;
    this->_debug = debug;
}

void MHSensor::init()
{
    pinMode(_pin, INPUT);
}

uint16_t MHSensor::getAnalogRead()
{
    _adc = analogRead(this->_pin);
    return (uint16_t)_adc;
}

float MHSensor::getVoltage()
{
    uint16_t adc = this->getAnalogRead();
    _sensorVolt = (adc * _voltResolution) / (pow(2, _adcBitResolution) - 1);
    return _sensorVolt;
}

float MHSensor::readFromAnalog()
{
    float volt = this->getVoltage();
    if (volt < MIN_OUTPUT_SIGNAL)
        return 0.0f;

    float ppm = this->mapOutputVoltage(volt, MIN_OUTPUT_SIGNAL, MAX_OUTPUT_SIGNAL, this->_minMeasuring, this->_maxMeasuring);
    return ppm;
}

float MHSensor::mapOutputVoltage(float val, float inMin, float inMax, float outMin, float outMax)
{
    return (val - inMin) * ((outMax - outMin) / (inMax - inMin)) + outMin;
}

float MHSensor::readFromUART(HardwareSerial *serial_port)
{
    long ppmCH4;
    byte bufferRead[CH4_DATA_LENGTH];
    byte bufferCommand[] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};

    serial_port->write(bufferCommand, sizeof(bufferCommand));
    serial_port->flush();

    int bytesRead = serial_port->readBytes(bufferRead, CH4_DATA_LENGTH);
    if (bytesRead != CH4_DATA_LENGTH)
        return -1;

    if ((bufferRead[0] == 0xFF) && (bufferRead[1] == 0x86))
    {
        if (_debug)
        {
            Serial.println("[INFO] voltage: " + String(getVoltage()));
            Serial.println("[INFO] Received bytes: " + (String)bytesRead);

            for (int i = 0; i < CH4_DATA_LENGTH; i++)
            {
                Serial.print(" - Byte");
                Serial.print(i);
                Serial.print(": ");
                Serial.println(bufferRead[i], HEX);
            }

            long unitIsPpm = ppmCH4 = ((int(bufferRead[2]) * 256) + int(bufferRead[3])) * 100;
            Serial.print("[INFO] Byte2 = " + String(bufferRead[2]) + ", Byte3 = " + String(bufferRead[3]));
            Serial.print(", Unit is ppm = ");
            Serial.println(unitIsPpm);
        }

        int decimal2 = bufferRead[2];
        int decimal3 = bufferRead[3];
        ppmCH4 = (decimal2 * 256 + decimal3) * 100;

        if (ppmCH4 <= 0)
            return 0.0;
        return (float)ppmCH4;
    }
    return -1;
}

boolean MHSensor::calibrateToZeroPoint(HardwareSerial *serial_port)
{
    byte bufferMH440D[CH4_DATA_LENGTH];
    byte MH440DCalibrationCommand[] = {0xFF, 0x01, 0x87, 0x00, 0x00, 0x00, 0x00, 0x00, 0x78};

    if (_debug)
        Serial.println("[INFO] Waiting Calibration MH440D...");

    if (serial_port->availableForWrite() < CH4_DATA_LENGTH)
    {
        if (_debug)
            Serial.println("[WARNING] Failed to Calibration. Retry!");
        return false;
    }
    serial_port->write(MH440DCalibrationCommand, sizeof(MH440DCalibrationCommand));
    serial_port->flush();
    return true;
}