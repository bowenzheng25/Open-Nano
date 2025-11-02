#ifndef PIN_MACROS_D51_H_
#define PIN_MACROS_D51_H_

#define PIN_BM(pin) (uint32_t)(1 << pin)
#define PIN_HI(port, pin) PORT->Group[port].OUTSET.reg = PIN_BM(pin) 
#define PIN_LO(port, pin) PORT->Group[port].OUTCLR.reg = PIN_BM(pin) 
#define PIN_TGL(port, pin) PORT->Group[port].OUTTGL.reg = PIN_BM(pin)
#define PIN_SETUP_OUTPUT(port, pin) PORT->Group[port].DIRSET.reg = PIN_BM(pin) 
#define PIN_SETUP_INPUT(port, pin) PORT->Group[port].DIRCLR.reg = PIN_BM(pin); PORT->Group[port].PINCFG[pin].reg = PORT_PINCFG_INEN
#define PIN_SETUP_PULLEN(port, pin) PORT->Group[port].PINCFG[pin].bit.PULLEN = 1
#define PIN_SETUP_PMUXEN(port, pin) PORT->Group[port].PINCFG[pin].bit.PMUXEN = 1

// WARNING: pretty sure abt this but haven't tested it... 
#define PIN_READ(port, pin) (PORT->Group[port].IN.reg & PIN_BM(pin))

#endif 