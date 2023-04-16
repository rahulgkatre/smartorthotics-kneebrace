#include <Adafruit_PCF8591.h>
// Make sure that this is set to the value in volts of VCC
#define ADC_REFERENCE_VOLTAGE 5.0
Adafruit_PCF8591 pcf = Adafruit_PCF8591();

void pcf8591Setup() {
  Serial.begin(115200);
  while (!Serial)
    delay(10);

  Serial.println("# Adafruit PCF8591 demo");
  if (!pcf.begin()) {
    Serial.println("# Adafruit PCF8591 not found!");
    while (1)
      delay(10);
  }
  Serial.println("# Adafruit PCF8591 found");
  pcf.enableDAC(true);

  Serial.println("AIN3");
//  last_reading = millis();
}

//unsigned long last_reading_pcf = 0;

void pcf8591Loop() {
  // Make a triangle wave on the DAC output
//  unsigned long curr = millis();
//  if (PRINT_PCF && curr - last_reading_pcf >= UPDATE_RATE_MS - UPDATE_RATE_CORRECTION) {
//    Serial.print(int_to_volts(pcf.analogRead(3), 8, ADC_REFERENCE_VOLTAGE));
//    Serial.println("");
//  }
}

float int_to_volts(uint16_t dac_value, uint8_t bits, float logic_level) {
  return (((float)dac_value / ((1 << bits) - 1)) * logic_level);
}
