#ifndef MCR_AD_LIB_H
#define MCR_AD_LIB_H

#include "Arduino.h"

#define AD_000 (R_ADC0->ADDR_b[0].ADDR)
#define AD_001 (R_ADC0->ADDR_b[1].ADDR)
#define AD_002 (R_ADC0->ADDR_b[2].ADDR)
#define AD_003 (R_ADC0->ADDR_b[3].ADDR)
#define AD_004 (R_ADC0->ADDR_b[4].ADDR)
#define AD_005 (R_ADC0->ADDR_b[5].ADDR)
#define AD_006 (R_ADC0->ADDR_b[6].ADDR)
#define AD_007 (R_ADC0->ADDR_b[7].ADDR)
#define AD_008 (R_ADC0->ADDR_b[8].ADDR)
#define AD_009 (R_ADC0->ADDR_b[9].ADDR)
#define AD_010 (R_ADC0->ADDR_b[10].ADDR)
#define AD_011 (R_ADC0->ADDR_b[11].ADDR)
#define AD_012 (R_ADC0->ADDR_b[12].ADDR)
#define AD_013 (R_ADC0->ADDR_b[13].ADDR)
#define AD_014 (R_ADC0->ADDR_b[14].ADDR)
#define AD_016 (R_ADC0->ADDR_b[16].ADDR)
#define AD_017 (R_ADC0->ADDR_b[17].ADDR)
#define AD_018 (R_ADC0->ADDR_b[18].ADDR)
#define AD_019 (R_ADC0->ADDR_b[19].ADDR)
#define AD_020 (R_ADC0->ADDR_b[20].ADDR)
#define AD_021 (R_ADC0->ADDR_b[21].ADDR)
#define AD_022 (R_ADC0->ADDR_b[22].ADDR)
#define AD_023 (R_ADC0->ADDR_b[23].ADDR)
#define AD_024 (R_ADC0->ADDR_b[24].ADDR)
#define AD_025 (R_ADC0->ADDR_b[25].ADDR)

class mcr_ad {
  public:
    void useCh( uint8_t ch );
    void start( void );
    int16_t getData( uint8_t ch );
    int16_t getDataDual( uint8_t ch );
    void scanDual( const uint8_t *ch, int16_t *ret, uint8_t num );
};

#endif // MCR_AD_LIB_H
