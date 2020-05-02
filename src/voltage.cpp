#include <Arduino.h>
#include <CayenneLPP.h>

const byte VOLTAGE_PIN = 35; // Prefer Use of ADC1 (8 channels, attached to GPIOs 32 - 39) . ADC2 (10 channels, attached to GPIOs 0, 2, 4, 12 - 15 and 25 - 27)
const byte ADC_BITS = 12;    // 10 - 12 bits

CayenneLPP lpp(51);
float voltage = 0;
float height = 0;
uint16_t adc_reading = 0;

float calcHeigth()
{
  float height = 48.7 * voltage - 17.3;
  return -height;
}

void initVoltage()
{
  analogReadResolution(ADC_BITS); // Default of 12 is not very linear. Recommended to use 10 or 11 depending on needed resolution.
  analogSetAttenuation(ADC_6db);  // Default is 11db which is very noisy. Recommended to use 2.5 or 6. Options ADC_0db (1.1V), ADC_2_5db (1.5V), ADC_6db (2.2V), ADC_11db (3.9V but max VDD=3.3V)
  pinMode(VOLTAGE_PIN, INPUT);
}

void readSensorValues()
{
  adc_reading = analogRead(VOLTAGE_PIN);
  voltage = adc_reading * 2.20 / (1 << ADC_BITS); // 3.9V because of 11dB, 100K/100K Voltage Divider, maxResolution (1024/2048/4096)
  height = calcHeigth();

  lpp.reset();
  lpp.addAnalogInput(1, voltage);
  lpp.addTemperature(2, height);
}
