#include "esp32_async_adc.h"
#include "soc/sens_reg.h"

bool ESP32AsyncADC::adcStart(uint8_t pin) {

  int8_t channel = digitalPinToAnalogChannel(pin);
  if (channel < 0) {
    return false; //not adc pin
  }

  if (channel > 9) {
    channel -= 10;
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD, (1 << channel), SENS_SAR2_EN_PAD_S);
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_SAR_M);
  } else {
    CLEAR_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
    SET_PERI_REG_BITS(SENS_SAR_MEAS_START1_REG, SENS_SAR1_EN_PAD, (1 << channel), SENS_SAR1_EN_PAD_S);
    SET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_START_SAR_M);
  }
  return true;
}

bool ESP32AsyncADC::adcBusy(uint8_t pin) {

  int8_t channel = digitalPinToAnalogChannel(pin);
  if (channel < 0) {
    return false;//not adc pin
  }

  if (channel > 7) {
    return (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0);
  }
  return (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0);
}

uint16_t ESP32AsyncADC::adcEnd(uint8_t pin) {
  uint16_t value = 0;
  int8_t channel = digitalPinToAnalogChannel(pin);
  if (channel < 0){
    return 0;//not adc pin
  }
  if(channel > 7){
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DONE_SAR) == 0); //wait for conversion
    value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_DATA_SAR, SENS_MEAS2_DATA_SAR_S);
  } else {
    while (GET_PERI_REG_MASK(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DONE_SAR) == 0); //wait for conversion
    value = GET_PERI_REG_BITS2(SENS_SAR_MEAS_START1_REG, SENS_MEAS1_DATA_SAR, SENS_MEAS1_DATA_SAR_S);
  }

  // Shift result if necessary
  uint8_t from = __analogWidth + 9;
  if (from == __analogReturnedWidth) {
    return value;
  }
  if (from > __analogReturnedWidth) {
    return value >> (from - __analogReturnedWidth);
  }
  return value << (__analogReturnedWidth - from);
}
