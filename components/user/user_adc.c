/***************************************************/
/*****************adc application*******************/
/************power value,co value,no2 value*********/
/***************************************************/


//header include
#include "stdint.h"
#include "adc.h"
#include "rom_map.h"
#include "hw_memmap.h"


double power_adcValue(void)
{
  uint8_t uiIndex=0;
  uint32_t val;
  uint32_t ulSample;
  uint32_t pulAdcSamples[14];
  double value;
  // Configure ADC timer which is used to timestamp the ADC data samples
  MAP_ADCTimerConfig(ADC_BASE,2^17);
  // Enable ADC timer which is used to timestamp the ADC data samples
  MAP_ADCTimerEnable(ADC_BASE);
  // Enable ADC module
  MAP_ADCEnable(ADC_BASE);
  // Enable ADC channel
  MAP_ADCChannelEnable(ADC_BASE, ADC_CH_1);
  while(uiIndex<14)
  {
    if(MAP_ADCFIFOLvlGet(ADC_BASE, ADC_CH_1))
    {
      ulSample = MAP_ADCFIFORead(ADC_BASE, ADC_CH_1);
      pulAdcSamples[uiIndex++] = ulSample;
    }
  }
  //Disable the adc channel
  MAP_ADCChannelDisable(ADC_BASE, ADC_CH_1);
  uiIndex =4;
  while(uiIndex<14)
  {
    val+=(pulAdcSamples[uiIndex++] >> 2 ) & 0x0FFF;
  }
  value=((double)val/10)*1.467/4096;
  return value; 
}

double CO_adcValue(void)
{
  uint8_t uiIndex=0;
  uint32_t val;
  uint32_t ulSample;
  uint32_t pulAdcSamples[14];
  double value;
  // Configure ADC timer which is used to timestamp the ADC data samples
  MAP_ADCTimerConfig(ADC_BASE,2^17);
  // Enable ADC timer which is used to timestamp the ADC data samples
  MAP_ADCTimerEnable(ADC_BASE);
  // Enable ADC module
  MAP_ADCEnable(ADC_BASE);
  // Enable ADC channel
  MAP_ADCChannelEnable(ADC_BASE, ADC_CH_3);
  while(uiIndex<14)
  {
    if(MAP_ADCFIFOLvlGet(ADC_BASE, ADC_CH_3))
    {
      ulSample = MAP_ADCFIFORead(ADC_BASE, ADC_CH_3);
      pulAdcSamples[uiIndex++] = ulSample;
    }
  }
  //Disable the adc channel
  MAP_ADCChannelDisable(ADC_BASE, ADC_CH_3);
  uiIndex =4;
  while(uiIndex<10)
  {
    val+=(pulAdcSamples[uiIndex++] >> 2 ) & 0x0FFF;
  }
  value=((double)val/10)*1.467/4096;
  return value/0.0014;        //(35000*0.00000004),gain 35k,40nA/ppm,max 1000ppm
}

double NO2_adcValue(void)
{
  uint8_t uiIndex=0;
  uint32_t val;
  uint32_t ulSample;
  uint32_t pulAdcSamples[14];
  double value;
  // Configure ADC timer which is used to timestamp the ADC data samples
  MAP_ADCTimerConfig(ADC_BASE,2^17);
  // Enable ADC timer which is used to timestamp the ADC data samples
  MAP_ADCTimerEnable(ADC_BASE);
  // Enable ADC module
  MAP_ADCEnable(ADC_BASE);
  // Enable ADC channel
  MAP_ADCChannelEnable(ADC_BASE, ADC_CH_2);
  while(uiIndex<14)
  {
    if(MAP_ADCFIFOLvlGet(ADC_BASE, ADC_CH_2))
    {
      ulSample = MAP_ADCFIFORead(ADC_BASE, ADC_CH_2);
      pulAdcSamples[uiIndex++] = ulSample;
    }
  }
  //Disable the adc channel
  MAP_ADCChannelDisable(ADC_BASE, ADC_CH_2);
  uiIndex =4;
  while(uiIndex<10)
  {
    val+=(pulAdcSamples[uiIndex++] >> 2 ) & 0x0FFF;
  }
  value=((double)val/10)*1.467/4096;
  return value/0.0084; //14000*0.0000006,14k gain ,600nA/ppm,max 150ppm
}

