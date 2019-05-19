//
// Get voltage of battery and shutdown on call if battery low
//
// Issues: ADC has probably some input inpedance and voltage reads constantly a bit low on TTGO T-Beam, it has 100k/100k divider at input
// This result in reading of about 3.9V with 4.1V battery on one test device.
//

#include <Arduino.h>

#include "globals.h"
#include "batvoltage.h"

#include "driver/adc.h" // these are for adc calibration
#include "esp_adc_cal.h"


void printBatteryVoltage(){
  Serial.print("Battery   : ");
  Serial.println(getBatteryVoltage(),3);
}


// Get battery voltage and if voltage low, shut down the device to avoid draining battery
// With TTGO T-beam other devices and internal divider continue to drain battery though. Need a protected 18650 definately
//
float getBatteryVoltage()
{
  // Voltage divider at pin 35 ie. ADC1_CHANNEL_8 with 100k/100k resistors
  const uint8_t samples=16;
  float voltage;

  analogReadResolution(12);           // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetWidth(12);                 // Sets the sample bits and read resolution, default is 12-bit (0 - 4095), range is 9 - 12 bits
  analogSetCycles(8); 
  analogSetSamples(4);
  analogSetAttenuation(ADC_6db);      // 2.2V full range on all ADCs
  adcAttachPin(35);
  analogSetClockDiv(255);

  //Characterize ADC at particular atten (factory calibration from fuses ued)
  esp_adc_cal_characteristics_t adc_chars;
  esp_adc_cal_value_t val_type = esp_adc_cal_characterize(ADC_UNIT_2, ADC_ATTEN_6db, ADC_WIDTH_BIT_12, 1100, &adc_chars);

  voltage = esp_adc_cal_raw_to_voltage(analogRead(35), &adc_chars)*2.0/1000; // this converts from millivolts, also handle 1:2 input divider

  // This handles uncalibrated reading. May be +-10% accuracy
  //voltage = 4.4*analogRead(35)/4096;  // convert to float, max adc voltage 2.2v, handle 1:2 divider at input too ie. full scale=4.4V

  // Shutdown at 3.2V (input impedance lowers reading, this is close to 3.5V in reality on test device)
  if(voltage < 3.2) {
    // TODO: Wait for transmissions to end? Lora module to sleep somehow? Something else to do?
    
    // gps to sleep
    const byte RXM_PMREQ[16] = {0xb5, 0x62, 0x02, 0x41, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x4d, 0x3b}; //power off until wakeup call
    gpsPort.write(RXM_PMREQ, sizeof(RXM_PMREQ)); // gps to sleep

    Serial.print("Shutting down, voltage ");
    Serial.print(voltage,3);
    Serial.flush();
    
    esp_deep_sleep_start();    // CPU to shutdown, only reset get out of here
  }

  return voltage;
  
}
