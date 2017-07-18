#define VERSION_MAJOR 10
#define VERSION_MINOR 12

// select one of these for control of output amplitudes:
//#define DIGIPOTS
#define DACS

/* beginnings of lock-in ampifier 
 
 ## we'd rather avoid frequencies that have periods that are even multiples of 160 clock ticks.
 - it gives some little artifacts in amp and phase. Only matters at high frequencies.
 though perhaps this is only with square wave input? Will need to test.
 
 10_12 replace I2C digipots. This is nearly Kais' last version, 
 with some minor fixes: ensures input capture buffer is emptied
 and  fixes match at halfway point of OC1
 
 
 10_8 rip out I2C stuff, replace with simple DAC. Move gain
 controls to other side of the board.
 pins 26-33 are 8 bit DAC output
 pin 34 is A/B for two DACS of AD7528 or TLC7528
 pin 35 is nWR
 
 on gain side, use pins A06-A11 for gain, pin A05 for latch
 
 10_7 - fix some numerical issues in / vs >> introduced recently
 10_6 minor aesthetics on input capture fudge - just set new_freq 
 to 2 and let it update pr2max's friends twice. Phase of sclock conquered -
 on input capture reset every period.
 Add version reporting instruction 'V'
 
 10_5 - rework input capture some more.  Looks like
 There are some issues with mpide-20120903 that screw up input
 capture. Very weird. value of new freq not always correct when coming
 out of the input capture interrupt.
 
 There's some real wackyness here in deb2. Setting the frequency circularly
 through deb2 seems to help. Can't for the life of me figure out why its
 needed.
 
 10_4 - have been working on input capture. Works pretty well, but the
 intermediate case - where the frequency changes by a medium size amount is flakey. 
 in 10_2 rework headers. Plan: header byte is a bit-field.
 bit 0/1 = capture_status
 2 - missed an interrupt
 3 - input overload
 4,5,6 - future use
 7 - next byte is a data identifier ( = 2 for new_phase, 3 for new_freq
 4 for deb1, 5 for deb 2, 6 for ldeb1, 7 for ldeb2, 8 for new_prescale, 9 is version #
 10 is for new averaged freq.)
 then 2 or 4 bytes for the value
 if bit 7 isn't set, then 4 bytes for the sample
 
 in 10_1 input capture is mostly fixed - still need to get phase correct with frequency drift, and
 relock on large freq changes
 
 ver 9: some cleanups/optimizations
 
 ver 8: try 4th order RC filter in second stage.
 
 ver7: add 6th order term to sin/cos calc. Good! Still pretty fast.
 
 ver 6: catch up with 6b, but with old sin/cos still to do: double check new sclock scheme
 
 ver 5: add freq measurement and start to clean up todo's
 
 ver 4: add input capture
 
 ver 3: start to add communications- appears to be working!
 
 Some gotchas: there are several (all removed now?) places in the data handling we used >> bit shift operators on signed
 integer values. These are no good!
 
 
 TODO: 
 
 - input capture - with frequency measurement over configurable number of periods
 - use timers 2 + 3 together as single 32 bit timer for low freq and high res.
 
 cic filters implemented in stand-alone program for test purposes
 in new_cic.c
 
 Our adcinterrupts will occur every 16 us at 500,000 sample rate.
 
 ver 2: add sin/cos approx calcs
 
 We actually have 10+3.3 bits of precision.
 
 PINS: input on A0 = RB2 = AN2
 output  oscillator on OC2 = RD1 = D5
 testing oscillator on OC1 = RD0 = D3
 input capture on IC1 = RD8 = D2
 
 */

#include <p32xxxx.h>
#include <plib.h>
#include <math.h>
#include <proc/p32mx320f128h.h>
#define __32MX320$128H__ 1


volatile int cn_proc=0; // output data ready flag.
int proc_count = 0; // how many have been processed in loop and uploaded.


volatile int overflows,new_prescale;
volatile unsigned short new_freq=0,new_phase=0,new_average=0;
volatile unsigned int total_counts,ntoaverage,naverages,ntotal_counts;
// ntotal_counts is the value we send upstream.

//unsigned short capture_interval;
// overflows is incremented each  time timer2 overflows
// capture interval is the number of counts between input captures.
// new_freq will be set if we update the frequency
// new_phase will be set if we update the phase.
volatile unsigned int capture_count; // the timer count at capture.
volatile int capture_status = 0;
volatile unsigned short deb1 = 0,deb2=0;
volatile unsigned int ldeb1=0,ldeb2=0;
volatile int input_overload=0;
unsigned int dsclk = 160;

//capture_status: 0 means using internal clock, 
// 1 means we're looking for first edge, 2 means looking for 2nd edge
// 3 means we think we're locked.

// for fast sin/cos
#define A  1.233537049
#define B -0.2526948331
int32_t ai,bi,ci;
int16_t shift2,shift4,shift6;

// timer prescaler
volatile uint16_t prescale;//in bits to shift
// variables for CIC2 filter:

int32_t shift_cic2;
//double growth,scale;

unsigned int mnum = 0; // index of cycle in second stage

unsigned int mdec2 = 8,mdec5 = 12; // decimation factor in CIC2, CIC5
int8_t start_timer_flag=0;


volatile unsigned short pr2max=20000; // this is a full period
volatile int32_t i5out,q5out;
unsigned short pr2max2=10000; // this is a half period.
unsigned short pr2max4 =5000; // quarter period
//volatile unsigned int tcic2,tcic5,dt2;

int32_t i5save,q5save;

char status = 0; // current state. 0 = not running but ready
// 1 is running, -1 is not configured.  For now, just 0/1
float aa,bb,cc;
int k,mk;

#ifdef DIGIPOTS
int I2C1_Set(unsigned short data)
{

  unsigned char L_Address = 0b0101111;

  I2C1CONbits.SEN = 1;    // ** Start condition enable
  //PORTFbits.RF0 = 1;
  while(I2C1CONbits.SEN); // what till it is cleared by module
  //PORTFbits.RF0 = 0;
  I2C1TRN =(L_Address << 1); // write is low
  while(I2C1STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C1STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C1TRN = data>>8;
  while(I2C1STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C1STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C1TRN = data;
  while(I2C1STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C1STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C1CONbits.PEN = 1;    // master issue stop command

  return 0;
}
//Digipot 2 
int I2C2_Set1( unsigned short data )
{

  unsigned char L_Address = 0b0101100;

  I2C2CONbits.SEN = 1;    // ** Start condition enable
  //PORTFbits.RF0 = 1;
  while(I2C2CONbits.SEN); // what till it is cleared by module
  //PORTFbits.RF0 = 0;
  I2C2TRN = L_Address << 1; // write is low
  while(I2C2STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C2STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C2TRN = data>>8;
  while(I2C2STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C2STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C2TRN = data;
  while(I2C2STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C2STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C2CONbits.PEN = 1;    // master issue stop command

  return 0;

}
int I2C2_Set( unsigned short data ){

  unsigned char L_Address = 0b0101111;

  I2C2CONbits.SEN = 1;    // ** Start condition enable
  //PORTFbits.RF0 = 1;
  while(I2C2CONbits.SEN); // what till it is cleared by module
  //PORTFbits.RF0 = 0;
  I2C2TRN = L_Address << 1; // write is low
  while(I2C2STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C2STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C2TRN = data>>8;
  while(I2C2STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C2STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C2TRN = data;
  while(I2C2STATbits.TRSTAT); // what until the nineth closck edge
  if( I2C2STATbits.ACKSTAT ) // ack not received
  {
    PORTFbits.RF0 = 1;
    return 1;
  }
  I2C2CONbits.PEN = 1;    // master issue stop command

  return 0;

}

#endif

void setup(){
  /* for some reason this seems to do more if its at the end of setup?
   // performance tuning:
   SYSTEMConfigPerformance(80000000L);
   mBMXDisableDRMWaitState();
   CheKseg0CacheOn();
   mCheConfigure(CHECON | 0x30);
   */

  /* set-up gain and dac bits as outputs: */
  for (k = 26 ; k <= 35 ; k++ ){
    pinMode(k,OUTPUT); // initialize to zero:
    digitalWrite(k,LOW);
  }
  digitalWrite(35,HIGH); // latch DACA 0
  // that wrote the first DAC as 0, now write second

  digitalWrite(34,HIGH);
  digitalWrite(35,LOW);
  digitalWrite(35,HIGH); // latch DACB as 0

  pinMode(A5,OUTPUT);
  pinMode(A6,OUTPUT);
  pinMode(A7,OUTPUT);
  pinMode(A8,OUTPUT);
  pinMode(A9,OUTPUT);
  pinMode(A10,OUTPUT);
  pinMode(A11,OUTPUT);

  // write the gain to be zero
  digitalWrite(A6,LOW);
  digitalWrite(A7,LOW);
  digitalWrite(A8,LOW);
  digitalWrite(A9,LOW);
  digitalWrite(A10,LOW);
  digitalWrite(A11,LOW);
  // toggle the latch
  digitalWrite(A5,LOW);
  digitalWrite(A5,HIGH);

  // write the amplitude to be zero:


  // disable all interrupts except those explicitly enabled
  // this include core timer - screws up millis and micros and delay etc. and digipots?
#ifndef DIGIPOTS
  IEC0 = 0;
  IEC1 = 0;
  OSCCONbits.PBDIV = 0;
#endif

  /* the chipkit mpide doesn't really know how to do 2megabaud.
   let it get set up first */
  Serial.begin(2000000);
#define _UARTMODE_BRGH 3
  // then fix the registers that it screwed up:
  U1BRG = ((__PIC32_pbClk / 4 /2000000)-1); // should be 9.
  U1MODE = (1<< _UARTMODE_ON ) | (1 << _UARTMODE_BRGH);
  // ok, now its actually set to 2 MBaud.

  ///////////////////////////////////////

  AD1CON1 = 0x0000;    //TURN ADC OFF
  AD1CON1 = 0xE0; // SSRC bit = 111 internal counter ends sample
  //  AD1CON1SET = 0X0500; // set signed 32bit output mode. 
  AD1CON1bits.FORM=5;// 5 for signed 32 bit
  AD1PCFG = ~(1 << 2);
  AD1CHS = (2 & 0xFFFF) <<16;  //CH0 POS INPUT IS AN2
  AD1CSSL = 0x0000;  //NO SCANNING

  /* there appears to be an error on the data sheet.  The total amount of time taken here
   seems to be (SAMC + 13) * Tad rather than + 12 */
  // this should be 500 ksps 8 cycles of sample + 12 cycles conversion at 10 MHz,
  AD1CON3 = 0x0703;  // set auto-sample to 8 Tad, Tad = 2 x (3+1)* TPB. 3 is 10MHz for adc clock
  //  AD1CON3 = 0x709;

  // slow everything down: use 1 MHz clock instead - 50 ksps
  //  AD1CON3 = 0x0700 + 39; // 1 MHz clock -> 50 ksps
  //  AD1CON3 = 0x0700 + 79; // .5 MHz clock -> 25 ksps
  //  AD1CON3 = 0x0700 + 199; // .2 MHz clock -> 10 ksps
  //  AD1CON3 = 0x0703;  // set auto-sample to 8 Tad, Tad = 2 x (3+1)* TPB. 3 is 10MHz for adc clock

  AD1CON2 = ((mdec2-1)<<2)+2;//3.3V Vref , interrupt every 5 acqs. turn on buffer fill mode.
  //   AD1CON2 = 0x07<<2; // don't use buffer fill mode, saves a branch later
  // clear my interrupt flag:
  IFS1CLR=0x2;

  // set priority to 7, subpriority to 3
  IPC6SET = 0x7000000<<2;
  IPC6SET = 0x3000000;
  // enable interrupt:
  IEC1SET = 0x2;
  AD1CON1SET = 0x8000;  //TURN ON ADC, auto start sample.
  ///////////////////////////////////////
  // set up oscillator using timer 2, output compare 1 (DIG 3 RD0)
  // this is using dual compare match mode.
  T2CON = 0x0;
  //  PR2 = 1; // 20 MHz 
  //  PR2 = 5; // 6.4 MHz should be
  //  PR2 = 400; // 200 kHz  counts to overflow.
  PR2 = pr2max-1; // for now use 40000 -> 2kHz
  //T2CONSET = 0x60; // prescale by 256
  prescale = 0; // this is for sin/cos multiply - set it here to match whatever goes into T2CON
  // its not perfect for prescale factors > 2^5 - depending on 
  // what's enabled below.
  // measured  in bits to shift.  to prescale, need to set TCKPS bits in T2CON 
  // these bits are in T2CON <<4.  000 = 1:1, 001 = 1:2, 010 = 1:4, 011 = 1:8,
  // 100 = 1:16, 101 = 1:32, 110 = 1:64, 111 = 1:256
  // TODO: need to set prescale. 

  TMR2 = 0;
  T2CONSET = 0x8000; // turn it on!
  ////////////////////////////////

  // set up output compare 2 to do square wave
  OC2R = 0x0; // match at start
  OC2RS = pr2max/2-1; // match at halfway point.
  OC2CON = 0x0005; // configure for OC2 pin low, dual match, source is timer 2.
  OC2CONSET = 0x8000; // enable OC2 module
  /////////////////////////////////

  // set up variables for sin/cos This section appears twice more: in 'F' and in input capture.
  // its kind of unnecessary here.
  shift2 = floor(2*log(pr2max4)/log(2.)+1-16);
  shift4 = floor(2*log((pr2max4*pr2max4)>>shift2)/log(2.)+1-16);
  shift6 = floor(log(((pr2max4*pr2max4)>>shift2))/log(2) +
    (2*log(((pr2max4*pr2max4)>>shift2))/log(2))-shift4+1-16);

  aa = A;
  bb = B;
  cc = 1.-aa-bb;

  aa = -aa/pr2max4/pr2max4;
  bb = -bb/pr2max4/pr2max4/pr2max4/pr2max4;
  cc = -cc/pr2max4/pr2max4/pr2max4/pr2max4/pr2max4/pr2max4;

  aa = aa*pow(2,shift2+30);
  bb = bb*pow(2,shift2+shift2+shift4+30);
  cc = cc*pow(2,shift2*2+shift4+shift2+shift6+30);

  ai = (int) round(aa);
  bi = (int) round(bb);
  ci = (int) round(cc);

  // scaling of CIC filter
  shift_cic2 = floor((2-0.5)*log(mdec2*1.0)/log(2.)) +1;// extra +1 to prevent overflows for low freq where upper sideband is present
  // this gives 5. We don't actually use this anymore, do a division, divide by 8 is hardcoded below.

  //  growth = powf(mdec2,2)*powf(mdec5,5); // 8 and 12 are decimations, 2 and 5 are order of CIC filters.
  //  scale = growth / powf(2.0,shift_cic2+shift_cic5)*4/2;// the four is for the extra two bits in sin/cos multiply
  // the two at the end is from the frequency translation - it divides everything by two in split to sum + diff freqs.
  // scale is the amount we divide by to get back to true input value.
  // these are no longer correct now that we do rc filter instead of CIC5.

  // variables for rc filter:
  // want y- = yi-1*k(xi-yi-1) k = dt/RC.  For the moment, dt at second stage is = 16us, want RC = 80us
  // so k is 0.2.  
  // to make math nicer, use k is 0.1875. this means  RC is 65.333us BW = 1865 Hz
#define RC_SHIFT 16
#define RC_SHIFTB 4
  k = 3;
  mk=16-k; // 16 is 2^RC_SHIFT
  // divide by 16.


  // speed ups:
  SYSTEMConfigPerformance(80000000L);
  // disable RAM wait state - should make things a tiny bit
  // faster, but means debugging is harder see 3.5.3 of family manual.
  //mBMXDisableDRMWaitState();
  // turn cache on - this helps!
  CheKseg0CacheOn();
  // turn on pre-fetch
  mCheConfigure(CHECON | 0x30);

  // may be possible to set flash wait sets to be faster using
  // something like:
  CHECONbits.PFMWS -=1; // (values 0 - 7)
  // though this is somewhat dangerous 7 is slowest?
  // hola! one extra tick off the flash wait states speeds things up tremendously!
  // take off 2 and it locks up...
  //OSCCONbits.PBDIV = 0b00;
}

void loop(){
  int sum = 0,val;
  static int t1,t2;
  unsigned char inb,header;
  short i5short,q5short,dummy;
  int i,j;
  static long idle_count=0;
  static unsigned char bytes_remaining=0,instruction_pending = 0;
  static unsigned char byte_buff[3];

  /*
list of current commands from serial
   'B': turn on adc and start sending data
   'E': stop ADC and stop sending data
   'Q': Serial.println("LCK READY");
   'A': set amplifer gain
   'F': set frequency
   'G': set frequency of other oscillator
   'C': two bytes for DAC A, DAC B to specifiy amplitude of output sine wave
   'I': start input capture
   'J': stop input capture
   'V': report version number 2 bytes: major then minor
   */

  if (Serial.available()){
    inb = Serial.read();
    if (bytes_remaining > 0){
      bytes_remaining--;
      byte_buff[bytes_remaining] = inb; // byes in the byte buffer come in backwards order? Why?
      if (bytes_remaining == 0){
        switch (instruction_pending){
        case 'F':
          unsigned short pre;
          //          T2CONCLR = 0x8070; // turn off, clear prescale bits. timer 3 can't prescale!
          // stop the ADC ( duplicates 'E')
          AD1CON1SET = 0x10; // set CLRASAM, so adc shuts off after next interrupt
          // read prescale value:
          prescale = byte_buff[2];   // this one byte is the number of bits to shift
          // should only take on values of 0, 1, 2, 3, 4, 5, 6, 8 // lowest freq = 4.7Hz
          pre = prescale - (prescale == 8); // if its 8, make it 7.
          T2CONbits.TCKPS =  pre; // turn on prescaling in hardware.

          // read timer count:
          pr2max = byte_buff[1]+(byte_buff[0]<<8);
          PR2 = pr2max-1; 
          OC2RS = pr2max/2-1; // match at halfway point.
          pr2max2= pr2max/2;
          pr2max4=pr2max/4; 
          if (TMR2 >= pr2max ) TMR2 = pr2max-2; // at -1 

          // set up variables for sin/cos
          shift2 = floor(2*log(pr2max4)/log(2.)+1-16);
          shift4 = floor(2*log((pr2max4*pr2max4)>>shift2)/log(2.)+1-16);
          shift6 = floor(log(((pr2max4*pr2max4)>>shift2))/log(2) +
            (2*log(((pr2max4*pr2max4)>>shift2))/log(2))-shift4+1-16);

          aa = A;
          bb = B;
          cc = 1.-aa-bb;

          aa = -aa/pr2max4/pr2max4;
          bb = -bb/pr2max4/pr2max4/pr2max4/pr2max4;
          cc = -cc/pr2max4/pr2max4/pr2max4/pr2max4/pr2max4/pr2max4;

          aa = aa*pow(2,shift2+30);
          bb = bb*pow(2,shift2+shift2+shift4+30);
          cc = cc*pow(2,shift2*2+shift4+shift2+shift6+30);

          ai = (int) round(aa);
          bi = (int) round(bb);
          ci = (int) round(cc);

          //          T2CONSET = 0x8000;
          // the rest duplicates 'B'
          if (status != 0){ // only restart if we were running
            i5out=0;
            q5out=0;
            status=1;
            proc_count = cn_proc;
            INTDisableInterrupts();
            //        TMR2 = 0;
            //        TMR3 = 0; // for consistent phase - but if input capture was set up, this wipes it out.
            //  AD1CON1SET = 0x8000;  //TURN ON ADC, auto start sample.
            AD1CON1CLR = 0x10; // clear CLRASAM
            AD1CON1SET = 0x4; // turn on auto sample bit, start up the adc
            TMR2 = 0; // how to fix phase of PB clock?
            INTEnableInterrupts();
            start_timer_flag = 1; // in case we were already running
          }

          break;
        case 'G':
          unsigned short pr3max,tprescale;
          tprescale = byte_buff[2];   // this one byte is the number of bits to shift
          pre = tprescale - (tprescale == 8); // if its 8, make it 7.
          T3CONbits.TCKPS =  pre; // turn on prescaling in hardware.
          // stop clock:
          //      T3CON = 0x0;
          //      T3CONCLR = 0x8070; // turn off, clear prescale bits (timer 3 can't prescale)

          // read prescale value:
          pr3max = byte_buff[1] + (byte_buff[0]<<8);
          PR3 = pr3max-1; 
          OC1CON = 0x000D; // dual match, continuous, from timer 3.
          OC1R = 0x0; // match at start
          OC1RS = pr3max/2-1; // match at halfway point.
          OC1CONSET = 0x8000;
          if (TMR3 >= pr3max ) TMR3 = pr3max-2;
          T3CONSET = 0x8000; // turn on.
          break;
        case 'A': // input amplifier gain.
          // these are pins A06 - A11
          LATBbits.LATB3 =byte_buff[0] & 01;
          LATBbits.LATB5 = (byte_buff[0] >> 1) & 0x1;
          LATBbits.LATB9 = (byte_buff[0] >> 2) & 0x1;
          LATBbits.LATB11 = (byte_buff[0] >> 3) & 0x1;
          LATBbits.LATB13 = (byte_buff[0] >> 4) & 0x1;
          LATBbits.LATB15 = (byte_buff[0] >> 5) & 0x1;
          // then toggle the latch.
          LATBbits.LATB14 = 0;  // pin A05
          LATBbits.LATB14 = 1;
          /*
          LATEbits.LATE0 = byte_buff[0] & 0x1;
           LATEbits.LATE1 = (byte_buff[0] >> 1) & 0x1;
           LATEbits.LATE2 = (byte_buff[0] >> 2) & 0x1;
           LATEbits.LATE3 = (byte_buff[0] >> 3) & 0x1;
           LATEbits.LATE4 = (byte_buff[0] >> 4) & 0x1;
           LATEbits.LATE5 = (byte_buff[0] >> 5) & 0x1;
           // bring VGA latch down first
           LATEbits.LATE7 = 0;
           // delay then bring the latch back up
           //          delay(1);  //delay doesn't work, core timer disabled
           LATEbits.LATE7 = 1; */
          break;
        case 'C':
#ifdef DIGIPOTS
          // write digipot 1
          //OSCCONbits.PBDIV = 0b11;
          unsigned short val = (byte_buff[0]>>8|byte_buff[1]<<2);
          //I2C1_Set( (1<<10) | 200); // TODO: do something with return value
          //I2C1_Set2( (1<<10) | 200 ); // TODO: do something with return value
          I2C2_Set(1<<10|val); // TODO: do something with return value 100K
          delay(2000);
          I2C2_Set1(1<<10|val); // TODO: do something with return value  20K
          //OSCCONbits.PBDIV = 0b00;
#endif
          // FIXME this needs a different letter code...
#ifdef DACS 
          // write DAC A
          LATDbits.LATD5 = 0; // A/B pin 34
          LATE = byte_buff[1]; //pins 26-33
          LATDbits.LATD11 = 0; // pin 35, nWR
          LATDbits.LATD11 = 1;
          // write DAC B
          LATDbits.LATD5 = 1;
          LATE = byte_buff[1];
          LATDbits.LATD11 = 0;
          LATDbits.LATD11 = 1;
#endif
          break;
        }// end switch
      } // end pending instructions
    } // end waiting for later bytes
    else{ // no bytes pending. This is a new instruction
      switch (inb){
      case 'B':
        if (status == 0){ // only do anything if we're not already running!
          i5out=0;
          q5out=0;
          status=1;
          proc_count = cn_proc;
          INTDisableInterrupts();
          //        TMR2 = 0;
          //TMR3 = 0; // for consistent phase - but if input capture was set up, this wipes it out.
          //  AD1CON1SET = 0x8000;  //TURN ON ADC, auto start sample.
          AD1CON1CLR = 0x10; // clear CLRASAM
          AD1CON1SET = 0x4; // turn on auto sample bit, start up the adc
          TMR2 = 0; // how to fix phase of PB clock?
          if (capture_status != 0){
            capture_status = 1; //restart capture
            IFS0CLR = 0x20; // reset interrupt flag for capture
            IFS0CLR = 0x100; // clear interrupt status flag of overflow 
            T2CONCLR = 0x70; // clear the prescale bits.
            overflows = 0;
            prescale = 0;
            PR2 = 65000;
          }
          else
            start_timer_flag = 1;
          INTEnableInterrupts();
        }
        break;
      case 'E':
        //      AD1CON1CLR = 0x8000; // Turn off adc
        AD1CON1SET = 0x10; // set CLRASAM, so adc shuts off after next interrupt
        status = 0;
        break;
      case 'Q':
        Serial.println("LCK READY");
        break;
      case 'A': // amplifier gain 
        instruction_pending = 'A';
        bytes_remaining = 1;
        break;
      case 'F':
        instruction_pending = 'F'; // main frequency
        bytes_remaining = 3;
        break;
      case 'G':
        instruction_pending = 'G'; // aux freq.
        bytes_remaining = 3;
        break;
      case 'C': // setting digi pots
        instruction_pending = 'C';
        bytes_remaining = 2;
        break;
      case 'D': // initializing  up digipots  
#ifdef DIGIPOTS
        OSCCONbits.PBDIV = 0b11;      // divided by 8;
        I2C2CONbits.ON = 0;     // disable I2C
        I2C2CONbits.SIDL = 0;   // continue module operation in Idle mode
        I2C2CONbits.STRICT = 0; // strick I2C reserved address rule no eabled
        I2C2CONbits.A10M = 0;   // AD5274 slave address is 7 bit
        I2C2CONbits.DISSLW = 0; // 0 -> Slew rate control enabled for High Speed mode
        // 1 -> Slew rate control disabled
        I2C2CONbits.SMEN = 0;   // disable SMBus spedific input, don't know what it is
        I2C2CONbits.ACKDT = 0;  // ** an not-ACK is sent at the end of a receive, but the master never receives
        I2C2CONbits.RCEN = 0;   // ** receive doesn't have to be enabled in master
        I2C2BRG = 500;
        TRISFbits.TRISF0 = 0;
        I2C2CONbits.ON = 1;     // turn on I2C module
        T1CON = 0x8030;

        I2C2_Set1( 0x1C02 ); 

        I2C2_Set( 0x1C02 ); 
        // disable write protect on RDAC  Kais commented out 
        PORTFbits.RF0 = 0;

        OSCCONbits.PBDIV = 0b11;      // divided by 8;
        I2C1CONbits.ON = 0;     // disable I2C
        I2C1CONbits.SIDL = 0;   // continue module operation in Idle mode
        I2C1CONbits.STRICT = 0; // strick I2C reserved address rule no eabled
        I2C1CONbits.A10M = 0;   // AD5274 slave address is 7 bit
        I2C1CONbits.DISSLW = 0; // 0 -> Slew rate control enabled for High Speed mode
        // 1 -> Slew rate control disabled
        I2C1CONbits.SMEN = 0;   // disable SMBus spedific input, don't know what it is
        I2C1CONbits.ACKDT = 0;  // ** an not-ACK is sent at the end of a receive, but the master never receives
        I2C1CONbits.RCEN = 0;   // ** receive doesn't have to be enabled in master
        I2C1BRG = 500;
        TRISFbits.TRISF0 = 0;
        I2C1CONbits.ON = 1;     // turn on I2C module
        T1CON = 0x8030;



        I2C1_Set( 0x1C02 ); 
        // disable write protect on RDAC  Kais commented out 
        PORTFbits.RF0 = 0;
#endif
        Serial.println("LCK READY");
        break;
      case 'I': // start input capture
        // todo: should turn off output until captured
        // todo: in general should only have output on if requested, and commands to turn on and off.
        IC1CON = 0; // clear capture control reg
        IC1CONSET = 0x0080; // use timer 2
        //      IC1CONSET = 0x0060; // interrupt every fourth capture event. No, 0 for every event
        IC1CONSET = 0x003; // capture every rising edge.
        IPC1SET = 0x1600; // priority 5, subpriority 2
        IEC0SET = 0x20; // enable interrupt on Input capture 1
        IFS0CLR = 0x20; // clear our interrupt status flag
        // enable capture:
        IC1CONSET = 0x8000; // enable bit
        /* read values from IC1BUF up to 4 reads.  bit 3 of IC1CON is set if there's data to read. */
        /* bit 5 of IFS0 is needs to be cleared before each interrupt. */
        /* because we aim for input capture interrupts to happen at 1/4 of PR, getting both input capture and overflow interrupts
         active at the same time likely means that the overflow came first, so give it higher priority */

        T2CONCLR = 0x70; // clear the prescale bits.
        prescale = 0;

        // enable interrupts on overflows
        overflows = 0;
        IPC2SET = 0x17 ;// set priority to 5, subpriority 3. 
        IFS0CLR = 0x100; // clear interrupt status flag.
        IEC0SET = 0x100; // enable the interrupt
        capture_status = 1;
        PR2 = 65000;
        // 0 is stopped, 1 means waiting for first edge. 2 means waiting for second edge, 3 means locked
        break;
      case 'J': // stop input capture
        IC1CONCLR = 0x8000; // turn off input capture
        IEC0CLR = 0x20; // stop input capture interrupts
        IEC0CLR = 0x100; // stop timer overflow interrupts
        capture_status = 0;
        start_timer_flag = 1; //gets phase back on.
        PR2 = pr2max-1;
        // tell the mothership what our current frequency is
        new_prescale = 1;
        new_freq = 1;
        break;
      case 'V':
        inb = 128;
        Serial.write(inb);  // a header byte: data coming
        inb = 9; // data is version
        Serial.write(inb);  // signal that more data is version info.
        inb = VERSION_MAJOR;
        Serial.write((const __uint8_t *) &inb,1);
        inb = VERSION_MINOR;
        Serial.write((const __uint8_t *) &inb,1);
        break;
      } // end switch     
    }

  } // end if serial.available

  // start main data servicing/upload:
  idle_count += 1;
  if (proc_count != cn_proc && status == 1){
    inb = capture_status;//  | (missedint > 0)*4; 
    if (input_overload != 0){
      inb |= 1<<3;
      input_overload = 0;
    }
    // low two bits are capture status. bit 2 is missed interrupt, bit 3 will be input overload
    Serial.write(inb);  // a header of one byte of zeros. 
    i5short = i5out;
    Serial.write((const __uint8_t *) &i5short,2);
    q5short = q5out;
    Serial.write((const __uint8_t *) &q5short,2);
    proc_count += 1;
    // this was 5 bytes in total.

    // this counts how many times we get through our idle loop - is a measure of how much spare computing capacity we have
    // Much of our time is taken in communications though.
    // each sample frame (5208.3 times/s), we transmit 8 bytes (5 data + 3 idle count), this takes 40 us.
    // we only have 192 us.
    // deb1 = idle_count; 
    idle_count = 0;
  }
  if (new_phase){
    inb = 128;
    Serial.write(inb);
    inb = 2; // header of 2 means new phase
    Serial.write(inb);
    Serial.write((const __uint8_t *) &capture_count,2);
    new_phase = 0;
  }
  if (new_prescale){
    inb = 128;
    Serial.write(inb);
    inb = 8;
    Serial.write(inb);
    Serial.write((const __uint8_t *) &prescale,2);
    new_prescale =0;
  }
  if (new_freq){
    inb = 128;
    Serial.write(inb);
    inb = 3; // header of 3 means new freq
    Serial.write(inb);

    Serial.write((const __uint8_t *) &pr2max,2);

    //    pr2max = capture_interval; // already done. Was needed for adc interrupp


    pr2max2= pr2max/2;
    pr2max4=pr2max/4; 
    // reset prefactors for sin/cos
    shift2 = floor(2*log(pr2max4)/log(2.)+1-16);
    shift4 = floor(2*log((pr2max4*pr2max4)>>shift2)/log(2.)+1-16);
    shift6 = floor(log(((pr2max4*pr2max4)>>shift2))/log(2) +
      (2*log(((pr2max4*pr2max4)>>shift2))/log(2))-shift4+1-16);

    aa = A;
    bb = B;
    cc = 1.-aa-bb;

    aa = -aa/pr2max4/pr2max4;
    bb = -bb/pr2max4/pr2max4/pr2max4/pr2max4;
    cc = -cc/pr2max4/pr2max4/pr2max4/pr2max4/pr2max4/pr2max4;

    aa = aa*pow(2,shift2+30);
    bb = bb*pow(2,shift2+shift2+shift4+30);
    cc = cc*pow(2,shift2*2+shift4+shift2+shift6+30);

    ai = (int) round(aa);
    bi = (int) round(bb);
    ci = (int) round(cc);

    // yeah, do it again. hitting this flag again here is 
    // a little ugly, but should help in phase consistency.
    new_freq -= 1;

  }
  if (new_average){
    inb = 128;
    Serial.write(inb);
    inb = 10;
    Serial.write(inb);
    Serial.write((const __uint8_t *) &ntoaverage,4); // number of periods we counted
    Serial.write((const __uint8_t *) &ntotal_counts,4); // total number of (prescaled) timer 
    new_average = 0;
  }
  if (deb1 != 0){
    inb = 128;
    Serial.write(inb);
    inb = 4;
    Serial.write(inb);
    Serial.write((const __uint8_t *) &deb1,2);
    deb1 = 0;
  }
  if (deb2 != 0){

    inb = 128;
    Serial.write(inb);
    inb = 5;
    Serial.write(inb);
    Serial.write((const __uint8_t *) &deb2,2);


    deb2 =0;
  }
  if (ldeb1 != 0){
    inb = 128;
    Serial.write(inb);
    inb = 6;
    Serial.write(inb);
    Serial.write((const __uint8_t *) &ldeb1,4);
    ldeb1 = 0;
  }
  if (ldeb2 != 0){
    inb = 128;
    Serial.write(inb);
    inb = 7;
    Serial.write(inb);
    Serial.write((const __uint8_t *) &ldeb2,4);
    ldeb2 =0;
  } 

}

uint32_t sclock;

#ifdef __cplusplus
extern "C" {
#endif


  void __ISR(_ADC_VECTOR,IPL7SRS) my_handler(void){
    unsigned int ic;
    //    static unsigned int t1,t2;
    unsigned int ci2c,ci4c,ci2s,ci4s;
    int isignc,isigns,valc,vals;
    int ipc,ippc;
    unsigned int lclock;
    int32_t *ndata;
    static int32_t i2[2],q2[2],id2[2][2],qd2[2][2];
    static int32_t irc[4],qrc[4];
    int32_t i2out,q2out;

    unsigned short lpr2max; //local copies. These are volatile and accesses are expensive
    // would also like to make a local copy of lprescale, but compile barfs trying to spill register.
    // is there another variable to reuse?

    /* this is the interrupt handler that deals with data acquired with the ADC.
     It frequency translates and does the downsampling and filtering
     */

    switch(start_timer_flag){
    case 1:
      TMR2 =  ((1276)>>prescale); // we should only need the %pr2max if freq is >= 62500
      // 1260 is 160*8 - 20 
      // 1278 is 160*8 - 2
      sclock = 0; 
      start_timer_flag = 0;
      break;
    case 2:
      sclock = TMR2;
      sclock = sclock<< prescale;
      sclock -= (1276-4); // the 4 is most of the difference between this and case 1.
      ic = pr2max;
      ic = ic<<prescale;
      sclock += ic; // to make sure sclock isn't negative.

      start_timer_flag = 0;
      break;
    }


    IFS1CLR = 0x02;// clear the interrupt flag 
    lpr2max = pr2max; // used because pr2max is volatile, and accessing it is expensive.

    //    t1 = micros();
    //   dt2 =t1-t2; // t2 will hold t1 from last time.

    // which half buffer is our data in:
    /*
  if (AD1CON2 & 0x80) { // data in first half
     ndata = (int *) &ADC1BUF0;
     // these are definitely right. 
     }
     else { // data in second half.
     ndata = (int *) &ADC1BUF8;
     }
     */
    ndata = (int32_t *) &ADC1BUF8;
    ndata -= (AD1CON2 & 0x80)>>2; // the buffers are 128 bytes apart


    // need to split into i and q
    // doing fixed decimation of 8.  So this only happens once at start.

    // this is done so that we can add it back on one extra time. The addition should be done
    // at the end of the loop. But the loop is much faster if we do the addition earlier in the loop.
    i2[1] -= i2[0];
    q2[1] -= q2[0];

    // now do the frequency mixing and integrating.
    // there is a phase shift in here from the actual clock - a phase shift of:
    // how long it takes to do 8 samples + how long it takes to read the clock.
    // should be able to correct for it.
    for (ic = 0;ic<8;ic++){


      // this is ok as long as pr2max > 8 * 160. (1280), max freq is 62500. Use the modulo above otherwise.
      /* sclock is measured in 80 MHz ticks, even though TMR2 might be measured in longer ticks, if prescaled.
       lclock is the prescaled version. 
       If pr2max < 1280, then clock could overflow twice, and my fake modulo below won't give the right answer. 
       
       */
      //      lclock = (sclock >> prescale)%pr2max;
      lclock = (sclock >> prescale);
      lclock = lclock - lpr2max*(lclock >= lpr2max);

      // calculate sin and cos.
      // intermingling seems to make a bit faster.
      ipc = lclock + pr2max4;

      // there are speed gains to be made by intermingling things better in here...

      i2[1] += i2[0];// left over from last time

      ippc = ipc%pr2max2;
      isignc = -2*((ipc >= pr2max2)  && (ipc < lpr2max)) +1;
      ippc -= pr2max4;

      ci2c = (ippc*ippc)>>shift2;
      ci4c = (ci2c*ci2c)>>shift4;
      valc = (ai*ci2c+bi*ci4c+1073741824); // goes to +/- 65536, just overflows 17 bits.
      ci4c = (ci2c*ci4c)>>shift6;
      valc += ci*ci4c;

      q2[1] += q2[0]; // left over from last time.
      valc = valc>>14; // this should be ok since valc should always be +ve here
      valc *= isignc;

      ipc = lclock - pr2max2*(lclock >= pr2max2) -pr2max4;


      valc *= *ndata;  


      isigns = -2*(lclock >= pr2max2) +1;
      ci2s = (ipc*ipc)>>shift2;
      ci4s = (ci2s*ci2s)>>shift4;
      vals = ai*ci2s+bi*ci4s+1073741824;
      ci4s = (ci2s*ci4s)>>shift6;
      vals += ci*ci4s;
      vals = vals>>14;
      vals *= isigns;

      valc /= (1<<13); // need speed? Change this back to valc = valc>>13, but loses a little accuracy.
      // valc =valc>>13; // it does lose some accuracy - don't do it!

      vals *= *ndata;
      vals /= (1<<13); // here too
      //vals = vals>>13;
      // the 160 is how many PB clock ticks between adc samples
      sclock += 160;

      q2[0] += valc;
      i2[0] += vals;
      //      q2[0] += (*ndata*vals)/(1<<13);
      //   q2[0] += (*ndata*vals)>>13;

      // samples are signed 10 bits, (511 -> -512)
      // sin/cos just overflow 17 bits (+/- 65536).
      // multiplied together, the product just overflows 26 bits.
      //after the shift by 13, we just overflow 13 bits.

      ndata += 4; // the registers are 16 bytes apart, who'd a thunk?

    }

    ic = lpr2max;
    ic = ic<<prescale;
    sclock = sclock %ic;

    i2[1] += i2[0];
    q2[1] += q2[0];

    // CIC2 differentiate
    id2[0][1] = id2[0][0];
    qd2[0][1] = qd2[0][0];

    id2[0][0] = i2[1];
    qd2[0][0] = q2[1];

    id2[1][1] = id2[1][0]; 
    qd2[1][1] = qd2[1][0];

    id2[1][0] = id2[0][1] - id2[0][0];
    qd2[1][0] = qd2[0][1] - qd2[0][0];

    i2out = (id2[1][1]-id2[1][0])<<RC_SHIFTB; // keep extra bits for rc filter precision.
    q2out = (qd2[1][1]-qd2[1][0])<<RC_SHIFTB;  
    //i2out = (id2[1][1]-id2[1][0])*RC_SHIFT; // keep extra bits for rc filter precision.
    //q2out = (qd2[1][1]-qd2[1][0])*RC_SHIFT;  

    // do rc filter:

    //    irc[0] = (k*i2out+ mk *irc[0])>>RC_SHIFT;
    //    qrc[0] = (k*q2out+ mk *qrc[0])>>RC_SHIFT;
    irc[0] = (k*i2out+ mk *irc[0])/RC_SHIFT;
    qrc[0] = (k*q2out+ mk *qrc[0])/RC_SHIFT;
    for (ic=1;ic<4;ic++){
      //      irc[ic] = (k*irc[ic-1]+ mk*irc[ic])>>RC_SHIFT;
      //      qrc[ic] = (k*qrc[ic-1]+mk*qrc[ic])>>RC_SHIFT;
      irc[ic] = (k*irc[ic-1]+ mk*irc[ic])/RC_SHIFT;
      qrc[ic] = (k*qrc[ic-1]+mk*qrc[ic])/RC_SHIFT;
    }

    mnum += 1;
    if (mnum == mdec5){
      mnum = 0;    
      //      i5out = irc[3]>>(RC_SHIFT+3); // here we divide out the RC shift that we added before the rc filter
      //      q5out = qrc[3]>>(RC_SHIFT+3); // also divide out the cic shift (5 bits) less two.
      i5out = irc[3]/RC_SHIFT/8; // here we divide out the RC shift that we added before the rc filter
      q5out = qrc[3]/RC_SHIFT/8; // also divide out the cic shift (5 bits) less two.
      cn_proc += 1;
    }

    //    t2=micros();
    //    tcic2=t2-t1;
    //    t2 = t1;

    // there's not really much point to this. If we're missing these interrupts, we're in serious trouble, and we'll 
    // never get the message anyway.
    //    missedint = (IFS1 & 0x02);
    ndata -= 4;
    // save the last sample to check for input overload - full range is -512 -> 511
    input_overload += (*ndata > 487 || *ndata < -487);
    //  input_sample = *ndata;

  }
#ifdef __cplusplus
}
#endif


// This interrupt handler is part of the input capture
#ifdef __cplusplus
extern "C" {
#endif
  void __ISR(_TIMER_2_VECTOR,IPL5) timer2_handler(void){
    IFS0CLR = 0x100; // clear timer interrupt status flag
    overflows += 1;
    // if reference signal is unplugged, we never get into the capture ISR,
    // flag the mother ship from here, and prep to re-capture
    // overflows<<prescale is a measure of the time since the last capture.

    if (((overflows<<prescale) > 513) && (capture_status == 3)){
      capture_status = 1; 
      T2CONCLR = 0x70; // clear the prescale bits.
      prescale = 0;
      overflows = 0;
      PR2 = 65000;
    }

  }
#ifdef __cplusplus
}
#endif


#ifdef __cplusplus
extern "C" {
#endif


  void __ISR(_INPUT_CAPTURE_1_VECTOR, IPL5) InputCapture1Handler(void)
  {
    int pre=0;
    unsigned short mtime,sinterval,mprescale;
    unsigned int now, new_time,minterval;
    // can "now" or "new_time" overflow?
    static unsigned short last_mtime;
    static short last_delta=0;
    short delta;
    int fdiff,nover;
    IFS0CLR = 0x20; // clear our interrupt status flag

    nover = overflows;
    overflows = 0; 
    while (IC1CON & 8)
      mtime = IC1BUF;
    capture_count = mtime; 
    //    new_phase = 1;
    // only need this if new_phase reports every period to the mothership    

    minterval = mtime+nover*(PR2+1)-last_mtime;
    // should do error checking and look at ICBNE (IC1CON<3>) to ensure there is data to see
    // and ICOV ICxCON<4> to make sure its not overrun 
    last_mtime = mtime;

    //todo: add a nice frequency measuring system: eg keep a ring buffer with a bunch of capture intervals
    // and a running sum.  After each capture, add in the new interval, subtract off the last from the ring buffer.
    // so we can measure the frequency with better accuracy that just with a single period.

    // for frequency averaging:
    total_counts += minterval;
    naverages += 1;
    if (naverages == ntoaverage){
      new_average = 1;
      ntotal_counts = total_counts;
      total_counts = 0;
      naverages = 0;
    }

    switch (capture_status){
    case 1:  // first edge - prescaling should have been turned off if this is true.
      capture_status += 1;
      //      deb1 = 1;
      return; 
    case 2: // second edge - this sets both the phase and frequency of our output.
      //      deb1 = 2;
      capture_status += 1;
      if (minterval > 65536){ // Assumes we had no prescaling at start. Start of input capture clears prescaler.
        mprescale = log(minterval)/log(2)-15;
        mprescale += (mprescale == 7);
        // so mprescale is the number of bits we prescale down by, needs to be 0,1,2,3,4,5,6,8
        // pre is the value that goes into the bits 0,1,2,3,4,5,6,7

          pre = mprescale - (mprescale == 8); // if its 8, make it 7.
        // prescale should only take on values of 0, 1, 2, 3, 4, 5, 6, 8 // lowest freq = 4.7Hz
        if (pre > 7){ // can't prescale that much.. should communicate this home todo
          capture_status = 2; // keep trying to lock
          return; // give up
        }
        prescale = mprescale; // alert the mothership that we're going to change the prescaler.
        minterval = minterval >> prescale; // set the number of timer counts down by the prescaler value.
      }
      new_prescale = 1; // always - since prescale might have been bigger before.
      // this reproduces stuff in 'F', could be consolidated.
      sinterval = minterval;
      // disable interrupts
      // want TMR2 to be pr2max4 at the next period         
      // time now:
      // in here pr2max and friends are the old values, not yet set.

      INTDisableInterrupts();
      now = TMR2;
      T2CONSET = pre<<4; // turn on prescaling.
      now = now + pr2max * (now < mtime);          // "now" is in full 80MHz cycles
      pr2max = sinterval; 
      PR2 = pr2max-1; // the adc interrupt may need this to be correct before the main loop sets it.
      OC2RS = pr2max/2-1;
      new_time = ((sinterval/4 +  ((now-mtime+28)>>prescale)) );  // the 28 is subject to change with the code just above
      new_time = new_time - sinterval * (new_time >= sinterval);
      TMR2 = new_time; 
      INTEnableInterrupts();

      start_timer_flag = 2;
      IFS1CLR = 0x02;// clear the interrupt adc interrupt flag in case it went off while we were in here.
      overflows = 0;
      IFS0CLR = 0x100; // clear interrupt status flag of overflow interrupt in case it happened.
      last_mtime = sinterval/4; // so that things look right for the next cycle
      // todo: force output to correct state (if output is turned on)? turn on if it should be?

      // reset pr2max'2 friends and sin/cos pre-calcs in main loop.

      new_freq = 2;

      // start averaging:
      total_counts = 0;
      naverages = 0;
      ntoaverage = (80000000>>prescale)/pr2max; // how many periods to average. How many periods in one second?
      new_average = 0;

      return;  
    case 3: // we believe we're locked already - prescaling is active
      // there are a number of different cases to consider here.
      // First: the input frequency is stable, but not exactly the same as our clock
      // basically, we don't mess the frequency here, just correct the phase.
      delta = mtime - pr2max4; // this is phase difference
      fdiff = minterval -pr2max; // this is freq (well, ok, period) difference
//ro-ee change
      if (fdiff < 10 && fdiff > -10){
        //        PR2 = PR2 + delta-last_delta; // this messes up sclock in the adc interrupt.  Repair sclock below:
        PR2 = pr2max-1+delta;
        /* what I am doing here is adjusting sclock to account for the wiggle in PR2 to keep locked.
         sclock is in 80MHz tics, so we subtract off (PR2+1-pr2max)<<prescale. But this might wrap sclock around (ie it goes negative)
         which would be bad, because it would then get screwed up in the adc interrupt.
         So add on another pr2max, then modulo it.
         */

        /* 
         // borrow variable "now"
         INTDisableInterrupts();
         now = PR2+1-pr2max-pr2max;
         sclock -= (now<<prescale); 
         
         now = pr2max;
         sclock = sclock %(now<<prescale); 
         INTEnableInterrupts();
         */
        // screw that, just set sclock again:
        start_timer_flag = 2;

        //        last_delta = delta*9/10;
        OC2RS = (PR2+1)/2-1; // match at halfway point.
        return;
      }
      // second case is that the new frequency appears to be wildly different from the old
      // here we give up and start over.

      // it would be nice to simplify this condition
      // this picks up anything that changes in either way by more than 1/4 period and also
      // when the frequency goes up enough that we should change prescaler.
      else if (nover != 1 || mtime > pr2max2 || (minterval <16000 && prescale >0)) // we're way off - either we didn't overflow, or we're off by more than 1/4 period

      {
        capture_status = 1; // start over.
        IFS0CLR = 0x100; // clear interrupt status flag of overflow 
        T2CONCLR = 0x70; // clear the prescale bits.
        overflows = 0;
        prescale = 0;
        PR2 = 65000;
        //        deb1 = 6;
        return;
      }

      // now, two other cases to consider: 3) steady drift of the frequency and 4) jump to a not-too-far-away frequency.
      // this *should* fix the frequency, but will leave the phase messed up. Phase will get fixed after the next period
      // by the first case above.

      else{ 
        pr2max = minterval;
        new_freq = 2;
        INTDisableInterrupts();
        PR2 = minterval-1;
        INTEnableInterrupts();
        OC2RS = sinterval/2-1; // match at halfway point.
        //        deb1 = 7;
        // start averaging:
        total_counts = 0;
        naverages = 0;
        ntoaverage = (80000000>>prescale)/pr2max; // how many periods to average. How many periods in one second?
        new_average = 0;

        if ( PR2 < TMR2) {
          TMR2 = PR2-2; // todo: this shouldn't ever happen, but is there a way to improve this?
          deb1 = 99;
          start_timer_flag = 2;
        }

        //    new_phase = 1;
        return;
      }

    } // end switch

  }// end ISR
#ifdef __cplusplus
}
#endif

































