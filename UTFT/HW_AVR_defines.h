#define cbi(reg, bitmask) *reg &= ~bitmask
#define sbi(reg, bitmask) *reg |= bitmask
#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);

#define cport(port, data) port &= data
#define sport(port, data) port |= data

#define pgm_read_word(data) *data
#define pgm_read_byte(data) *data

#define swap(type, i, j) {type t = i; i = j; j = t;}

//#define fontbyte(x) pgm_read_byte(&cfont.font[x])  
#define fontbyte(x) cfont.font[x]

//#define regtype volatile uint8_t
#define regtype volatile uint32_t
//#define regsize uint8_t
#define regsize uint32_t
#define bitmapdatatype unsigned int*
//#define bitmapdatatype unsigned uint16_t*


// *** Hardwarespecific defines ***
//#define pulse_high(reg, bitmask) sbi(reg, bitmask); cbi(reg, bitmask);
//#define pulse_low(reg, bitmask) cbi(reg, bitmask); sbi(reg, bitmask);

//#define cport(port, data) port &= data
//#define sport(port, data) port |= data

//#define swap(type, i, j) {type t = i; i = j; j = t;}

//#define fontbyte(x) pgm_read_byte(&cfont.font[x])  

//#define regtype volatile uint8_t
//#define regsize uint8_t
//#define bitmapdatatype unsigned int*
