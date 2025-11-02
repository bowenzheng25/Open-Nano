#include "timers.h"
#include "pin_macros.h"
#include "peripheral_nums.h"

#define PIN_TCC0_DEBUG 20 
#define PRT_TCC0_DEBUG 0 

void setup_tcc_0(void){
  // use TCC0 for very fast pwm out, und TCC1 for the other ch 
  MCLK->APBBMASK.bit.TCC0_ = 1; 
  MCLK->APBBMASK.bit.TCC1_ = 1; 

  // setup a generic clock for TCC0, 
  GCLK->PCHCTRL[TCC0_GCLK_ID].bit.GEN = GCLK_PCHCTRL_GEN_GCLK0_Val; // use GCLK0 (@ 48MHz?)
  GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN = 1; // enable ?
  while(!GCLK->PCHCTRL[TCC0_GCLK_ID].bit.CHEN);
  // 
  GCLK->PCHCTRL[TCC1_GCLK_ID].bit.GEN = GCLK_PCHCTRL_GEN_GCLK0_Val; // use GCLK0 (@ 48MHz?)
  GCLK->PCHCTRL[TCC1_GCLK_ID].bit.CHEN = 1; // enable ?
  while(!GCLK->PCHCTRL[TCC1_GCLK_ID].bit.CHEN);

  // configure an output pin to check (no need for TCC0)
  PIN_SETUP_PMUXEN(PRT_TCC0_DEBUG, PIN_TCC0_DEBUG);
  PORT->Group[PRT_TCC0_DEBUG].PMUX[PIN_TCC0_DEBUG >> 1].reg |= 
    (PIN_TCC0_DEBUG % 2 ? PORT_PMUX_PMUXO(PERIPHERAL_G) : PORT_PMUX_PMUXE(PERIPHERAL_G)); 

  // setup TCC0: reset and wait to clear 
  TCC0->CTRLA.bit.SWRST = 1;
  while(TCC0->SYNCBUSY.bit.SWRST);
  // setup TCC1
  TCC1->CTRLA.bit.SWRST = 1;
  while(TCC1->SYNCBUSY.bit.SWRST);

  // set prescaler and reload on gclk, this is baseline main_freq (120MHz default)
  TCC0->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_GCLK; 
  TCC1->CTRLA.reg = TCC_CTRLA_PRESCALER_DIV1 | TCC_CTRLA_PRESCSYNC_GCLK; 

  // we want... match frequency mode
  // in MFRQ, top = CC0 
  // and toggle will only come out on TCC0[0] (PA20, Periperal G) 
  TCC0->WAVE.reg = TCC_WAVE_WAVEGEN_MFRQ_Val;
  while(TCC0->SYNCBUSY.bit.WAVE);
  TCC1->WAVE.reg = TCC_WAVE_WAVEGEN_MFRQ_Val;
  while(TCC1->SYNCBUSY.bit.WAVE);

  // we'll use the output from this to increase count on the next 
  // nyet, dmac trig 
  // TCC0->EVCTRL.reg = TCC_EVCTRL_OVFEO;

  // init @ some period 
  write_periods(255, 255);

  // now en-able, 
  TCC0->CTRLA.bit.ENABLE = 1; 
  while(TCC0->SYNCBUSY.bit.ENABLE);
  TCC1->CTRLA.bit.ENABLE = 1; 
  while(TCC1->SYNCBUSY.bit.ENABLE);
}

void setup_dac(void){
  // arduino does the nuts and bolts, then we can just pipe to DATA
  pinMode(DAC0, OUTPUT);
  analogWriteResolution(12); 
  analogWrite(DAC0, 0);
  // 
  pinMode(DAC1, OUTPUT);
  analogWriteResolution(12);
  analogWrite(DAC1, 0);
}

// our dmac buffer ... 12 bits,
#define DMAC_WAVE_SIZE 4096
// we want two, and two descriptors, just to avoid witches in the DMAC 
uint16_t dmac_wave_fwd_0[DMAC_WAVE_SIZE]; // fwds dir 
uint16_t dmac_wave_fwd_1[DMAC_WAVE_SIZE]; // fwds dir 
uint16_t dmac_wave_rev_0[DMAC_WAVE_SIZE]; // rev dir 
uint16_t dmac_wave_rev_1[DMAC_WAVE_SIZE]; // rev dir 

// we allocate 32 each, *just in case* CH2 > is ever enabled, 
// it won't point at garbage and hard fault 
static DmacDescriptor descriptors[32] __attribute__((aligned(16)));
static DmacDescriptor wb_descriptors[32] __attribute__((aligned(16)));

void setup_dmac(void){
  // write vals 2 wave
  for(uint16_t i = 0; i < DMAC_WAVE_SIZE; i ++){
    dmac_wave_fwd_0[i] = i;
    dmac_wave_fwd_1[i] = i;
    dmac_wave_rev_0[i] = DMAC_WAVE_SIZE - 1 - i;
    dmac_wave_rev_1[i] = DMAC_WAVE_SIZE - 1 - i;
  }
  // setup dmac ... 
  // disable to config 
  DMAC->CTRL.bit.DMAENABLE = 0; 
  // descriptor addr, writeback addr 
  DMAC->BASEADDR.reg = (uint32_t)&descriptors;
  DMAC->WRBADDR.reg = (uint32_t)&wb_descriptors; 
  // turn on all levels 
  DMAC->CTRL.bit.LVLEN0 = 1;
  DMAC->CTRL.bit.LVLEN1 = 1;
  DMAC->CTRL.bit.LVLEN2 = 1;
  DMAC->CTRL.bit.LVLEN3 = 1;
  // re-enable 
  DMAC->CTRL.bit.DMAENABLE = 1; 
  // configure channel 0 TCC0 -> DAC0
  DMAC->Channel[0].CHCTRLA.bit.BURSTLEN = 0x0;  // single beat burst
  DMAC->Channel[0].CHCTRLA.bit.TRIGACT = 0x2;   // trigger per burst 
  DMAC->Channel[0].CHCTRLA.bit.TRIGSRC = TCC0_DMAC_ID_OVF; // trigger from TCC0 Overflow 
  DMAC->Channel[0].CHPRILVL.bit.PRILVL = 0x0;     // channel has 0 priority (lowest!)
  // configure channel 1 TCC1 -> DAC1
  DMAC->Channel[2].CHCTRLA.bit.BURSTLEN = 0x0;  // single beat burst
  DMAC->Channel[2].CHCTRLA.bit.TRIGACT = 0x2;   // trigger per burst 
  DMAC->Channel[2].CHCTRLA.bit.TRIGSRC = TCC1_DMAC_ID_OVF; // trigger from TCC0 Overflow 
  DMAC->Channel[2].CHPRILVL.bit.PRILVL = 0x1;     // channel has 1 priority 
  // set descriptor values... we need two to wrap, 
  // yikes, since we have to interleave... 
  descriptors[0].BTCTRL.bit.VALID = 0x1;            // descriptor is valid 
  descriptors[0].BTCTRL.bit.EVOSEL = 0x0;           // no events 
  descriptors[0].BTCTRL.bit.BLOCKACT = 0x0;         // no end of block action: disables apres 
  descriptors[0].BTCTRL.bit.BEATSIZE = 0x1;         // beatsize = 1 = HWORD = 2 bytes
  descriptors[0].BTCTRL.bit.SRCINC = 0x1;           // inc. source addr 
  descriptors[0].BTCTRL.bit.DSTINC = 0x0;           // don't increment destination 
  descriptors[0].BTCTRL.bit.STEPSEL = 0x1;          // step applies to src 
  descriptors[0].BTCTRL.bit.STEPSIZE = 0x0;         // step size is "X1" 
  descriptors[0].BTCNT.reg = DMAC_WAVE_SIZE;        // how many itemz 
  descriptors[0].SRCADDR.reg = (uint32_t)(dmac_wave_fwd_0 + DMAC_WAVE_SIZE);   // src starts here (pre decriment?)
  descriptors[0].DSTADDR.reg = (uint32_t)&(DAC->DATA[0].reg) ;            // hwords to dac 
  descriptors[1].BTCTRL.bit.VALID = 0x1;            // descriptor is valid 
  descriptors[1].BTCTRL.bit.EVOSEL = 0x0;           // no events 
  descriptors[1].BTCTRL.bit.BLOCKACT = 0x0;         // no end of block action: disables apres 
  descriptors[1].BTCTRL.bit.BEATSIZE = 0x1;         // beatsize = 1 = HWORD = 2 bytes
  descriptors[1].BTCTRL.bit.SRCINC = 0x1;           // inc. source addr 
  descriptors[1].BTCTRL.bit.DSTINC = 0x0;           // don't increment destination 
  descriptors[1].BTCTRL.bit.STEPSEL = 0x1;          // step applies to src 
  descriptors[1].BTCTRL.bit.STEPSIZE = 0x0;         // step size is "X1" 
  descriptors[1].BTCNT.reg = DMAC_WAVE_SIZE;        // how many itemz 
  descriptors[1].SRCADDR.reg = (uint32_t)(dmac_wave_fwd_0 + DMAC_WAVE_SIZE);   // src starts here (pre decriment?)
  descriptors[1].DSTADDR.reg = (uint32_t)&(DAC->DATA[0].reg) ;            // hwords to dac 
  // wrap 
  descriptors[0].DESCADDR.reg = (uint32_t)&(descriptors[1]);
  descriptors[1].DESCADDR.reg = (uint32_t)&(descriptors[0]);
  // ch2
  descriptors[2].BTCTRL.bit.VALID = 0x1;            // descriptor is valid 
  descriptors[2].BTCTRL.bit.EVOSEL = 0x0;           // no events 
  descriptors[2].BTCTRL.bit.BLOCKACT = 0x0;         // no end of block action: disables apres 
  descriptors[2].BTCTRL.bit.BEATSIZE = 0x1;         // beatsize = 1 = HWORD = 2 bytes
  descriptors[2].BTCTRL.bit.SRCINC = 0x1;           // inc. source addr 
  descriptors[2].BTCTRL.bit.DSTINC = 0x0;           // don't increment destination 
  descriptors[2].BTCTRL.bit.STEPSEL = 0x1;          // step applies to src 
  descriptors[2].BTCTRL.bit.STEPSIZE = 0x0;         // step size is "X1" 
  descriptors[2].BTCNT.reg = DMAC_WAVE_SIZE;        // how many itemz 
  descriptors[2].SRCADDR.reg = (uint32_t)(dmac_wave_rev_1 + DMAC_WAVE_SIZE);   // src starts here (pre decriment?)
  descriptors[2].DSTADDR.reg = (uint32_t)&(DAC->DATA[1].reg) ;            // hwords to dac 
  descriptors[3].BTCTRL.bit.VALID = 0x1;            // descriptor is valid 
  descriptors[3].BTCTRL.bit.EVOSEL = 0x0;           // no events 
  descriptors[3].BTCTRL.bit.BLOCKACT = 0x0;         // no end of block action: disables apres 
  descriptors[3].BTCTRL.bit.BEATSIZE = 0x1;         // beatsize = 1 = HWORD = 2 bytes
  descriptors[3].BTCTRL.bit.SRCINC = 0x1;           // inc. source addr 
  descriptors[3].BTCTRL.bit.DSTINC = 0x0;           // don't increment destination 
  descriptors[3].BTCTRL.bit.STEPSEL = 0x1;          // step applies to src 
  descriptors[3].BTCTRL.bit.STEPSIZE = 0x0;         // step size is "X1" 
  descriptors[3].BTCNT.reg = DMAC_WAVE_SIZE;        // how many itemz 
  descriptors[3].SRCADDR.reg = (uint32_t)(dmac_wave_rev_1 + DMAC_WAVE_SIZE);   // src starts here (pre decriment?)
  descriptors[3].DSTADDR.reg = (uint32_t)&(DAC->DATA[1].reg) ;            // hwords to dac 
  // wrap 
  descriptors[2].DESCADDR.reg = (uint32_t)&(descriptors[3]);
  descriptors[3].DESCADDR.reg = (uint32_t)&(descriptors[2]);

  // and turn the channel on, forever ? 
  DMAC->Channel[0].CHCTRLA.bit.ENABLE = 1; 
  DMAC->Channel[2].CHCTRLA.bit.ENABLE = 1; 
}

void setup_timers(void){
  setup_tcc_0();
  setup_dac();
  setup_dmac();
}


void write_periods(int16_t per_0, int16_t per_1){
  // dir, per... 
  if(per_0 < 0){
    descriptors[0].SRCADDR.reg = (uint32_t)(dmac_wave_rev_0 + DMAC_WAVE_SIZE);
    descriptors[1].SRCADDR.reg = (uint32_t)(dmac_wave_rev_0 + DMAC_WAVE_SIZE);
    TCC0->CCBUF[0].reg = abs(per_0);
  } else {
    descriptors[0].SRCADDR.reg = (uint32_t)(dmac_wave_fwd_0 + DMAC_WAVE_SIZE);
    descriptors[1].SRCADDR.reg = (uint32_t)(dmac_wave_fwd_0 + DMAC_WAVE_SIZE);
    TCC0->CCBUF[0].reg = abs(per_0);
  }
  // 
  if(per_1 < 0){
    descriptors[2].SRCADDR.reg = (uint32_t)(dmac_wave_rev_1 + DMAC_WAVE_SIZE);
    descriptors[3].SRCADDR.reg = (uint32_t)(dmac_wave_rev_1 + DMAC_WAVE_SIZE);
    TCC1->CCBUF[0].reg = abs(per_1);
  } else {
    descriptors[2].SRCADDR.reg = (uint32_t)(dmac_wave_fwd_1 + DMAC_WAVE_SIZE);
    descriptors[3].SRCADDR.reg = (uint32_t)(dmac_wave_fwd_1 + DMAC_WAVE_SIZE);
    TCC1->CCBUF[0].reg = abs(per_1);
  }
}