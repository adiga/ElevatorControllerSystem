//*****************************************************
// Project: Elevator controller
// Authors: Vikram Adiga/Sudersan Sampath
// Desc: Using HSC12, elevator controller is designed
//       This code contains the FSM logic of operation 
//       of elevator, the motor controller logic, reads 
//       inputs from keypad and ir sensors. PWM to 
//	 control the speed of motor.
// Note: The code for LCD is reused from the previous
//       lab assignments.LCD is used only for debugging
//*****************************************************
#include <hidef.h>      /* common defines and macros */
#include <mc9s12c32.h>     /* derivative information */
#pragma LINK_INFO DERIVATIVE "mc9s12c32"


#define ENABLE_BIT 0x80
#define RS_BIT 0x40

void LCDInit(void);
void LCDdelay(unsigned long ms);
void spiWR(unsigned char data);
void LCDWR(unsigned char data);
void LCDChar(unsigned char letter);
void LCDNum(int val);
void LCDClear(void);
void LCDCursorOn(void);
void LCDCursorOff(void);
void LCDString(char *pt);
void LCDDecimal(unsigned char val);
void LCDInt(unsigned int val);
void LCDHex(unsigned char val);

void Timer_Init(void);               // Timer Initialization 
void Timer_Wait10ms(void);           // Timer for delay  
void motorStop(void);		     // Stop motor for at each level

void IRQ_Init(void);                 // IRQ initialization
void interrupt 5 XIRQHan(void);      // XIRQ handler
void interrupt 6 IRQHan(void);       // IRQ handler
void Init(void);                     // Port initialization
int ReadInput(void);                 // Read input from PTT
void scan(void);                     // Pull each line and Scan the keypad 
void scanInput(int value);           // Scan and assign values for PTT
void scanIRSensor(void);             // Scan IR sensor
void motorController(void);          // Motor Controller logic.

unsigned volatile int level1 = 0;    // Button 1 (inside elevator) 0 : false , 1: true
unsigned volatile int level2 = 0;    // Button 2 (inside elevator) 0 : false , 1: true
unsigned volatile int level3 = 0;    // Button 3 (inside elevator) 0 : false , 1: true
unsigned volatile int up1 = 0;       // Up button (at level 1) (outside elevator) 0 : false , 1: true
unsigned volatile int down2 = 0;     // Down button (at level 2) (outside elevator) 0 : false , 1: true
unsigned volatile int up2 = 0;       // Up button (at level 2) (outside elevator) 0 : false , 1: true
unsigned volatile int down3 = 0;     // Down button (at level 3) (outside elevator) 0 : false , 1: true


unsigned volatile int button = 0;        // Current IR sensor. 1: level1, 2 : level2, 3: level3
unsigned volatile int currentstate = 1;  // State variable for FSM 
unsigned volatile int nextstate = 0;     // State variable for FSM
unsigned volatile int direction = 0;     // to control motor direction. 1: UP (clockwise), 2: DOWN (anticlockwise), 0: STOP


void PWM_Init(void);                      // PWM initializer
void PWM_Duty(unsigned char duty);        // Setting duty cycle for PWM

void main(void) {

  /*Initizaling*/
  Init();
  LCDInit();
  Timer_Init();
  LCDClear();
  PWM_Init();
  PWM_Duty(225);
  PTT=0x0F;
  PTAD=0x00;
  EnableInterrupts;
  asm ANDCC #$BF;       //Arm XIRQ
  IRQ_Init();
 
 
  for(;;) {}
}

//***********************************************************
// Internal PWM hardware is intialized to output on Port P5
//***********************************************************
void PWM_Init(void){
//PWM on PP5
PWME = PWME | 0x20; 		  //enable Channel 5
PWMPOL = PWMPOL | 0x20; 	  //PP5 intially high then low
PWMCLK = PWMCLK | 0x20;           //Clock SA for PP5
PWMPRCLK = (PWMPRCLK&0xF8) | 0x04; //Clock A = E Clock/16
PWMSCLA = 5;                      //Clock SA = Clock A/10    0.25 * 160 = 40us
PWMPER5 = 250;                    //10ms
PWMDTY5 = 0;                      //initially off
}

//**********************************************************
// Sets the duty cycle: Useful for setting different speeds
// in different directions of elevator.
//**********************************************************
void PWM_Duty(unsigned char duty){
PWMDTY5 = duty;                   // 0 to 250
}

//**********************************************************
// Timer Intialization for delay
//**********************************************************
void Timer_Init(void){
TIOS = 0x20;        //select TC5 
TSCR1 = 0X80;       //enable timer  
TSCR2 =0x04;        //set the prescale bits
}

//*********************************************************
// Provides timer delay
//*********************************************************
void Timer_Wait10ms(void){
TC5 = TCNT + 120000;        //set the end time    
TFLG1 = 0x20;               //Clear flag
while((TFLG1&0x20) ==0){
}                           //Wait till flag is set
}

//*********************************************************
// Stop the motor for some time. This is to make the the 
// elevator stop at a particular level before proceeding to 
// next level. 
//*********************************************************
void motorStop(void){
int i;
PTAD = (PTAD & ~(PTAD_PTAD7_MASK|PTAD_PTAD6_MASK)); // Stop motor

for(i =0 ; i< 25; i++){
    Timer_Wait10ms();
  }
}


//********************************************************
// Elevator controller FSM: based on the current IR sensed
// (button variable : actually a misnomer)the current 
// state and next state are then defined along with the 
// direction the motor has to run.
// This FSM also provides arbitration, when more than one
// level button is pressed.
//********************************************************

void motorController(void){

//Switch based on IR sensor: name "button" a misnomer

switch(button){

  case 1: if(button == currentstate){
            //reset current level vars
            level1 = 0;
            up1 = 0;
         
            //delay
            motorStop();
         
         
         
            //condition for level2
            if((level2 == 1) || (up2 == 1)){
              nextstate = 2;
              direction = 1;
            }
            //condition for level3
            else if ((level3 == 1) || (down3 == 1)){
              nextstate = 3;
              direction = 1;
            }else if (down2 == 1){
              nextstate = 2;
              direction = 1;
          
            }
            //condtion if nothing is pressed
            //should state in same level
            else{
              nextstate = currentstate;
              direction = 0;               
            }
       
          }
          break;
 
  case 2:  if(button == currentstate || level2 == 1 || up2 == 1 || down2 == 1){
            //reset current level vars
            level2 = 0;
         
            //delay
            motorStop();
         
            //condition for level1
            if(((level1 == 1) || (up1 == 1)) && !((down3 == 1) && (direction == 1))){
             nextstate = 1;
             direction = 2;
             down2 = 0; // reset
            }
            //condition for level2
            else if (((level3 == 1) || (down3 ==1)) && !((up1 == 1) && (direction == 2))){
              nextstate = 3;
              direction = 1;
              up2 = 0; // reset
            }
            //condtion if nothing is pressed
            //should state in same level
             else {
              nextstate = currentstate;
              direction = 0;
              up2 = 0;
              down2 = 0;
            }
           }
           break;
 
  case 3:  if(button == currentstate){
            //reset current level vars
            level3 = 0;
            down3 = 0;
         
            //delay
            motorStop();
         
            //condition for level2
            if((level2 == 1) || (down2 == 1)){
             nextstate = 2;
             direction = 2;
            }
            //condition for level1
            else if ((level1 == 1 ) || (up1 == 1)){
              nextstate = 1;
              direction = 2;
            } else if (up2  == 1){
             nextstate = 2;
             direction = 2;
          
            }
            //condtion if nothing is pressed
            //should state in same level
            else {
              nextstate = currentstate;
              direction = 0;
            }
           }
           break;
        
   default: //LCDClear();
            //LCDInt(button);
            //LCDString("IRErr");
            break;
         
                   
}


 
  if(direction == 1){
   //set the up direction duty cycle
   PWM_Duty(225);
   //Set the motor direction to move elevator UP 	
   PTAD = (PTAD & ~(PTAD_PTAD6_MASK))|(PTAD_PTAD7_MASK);
 
  } else if(direction == 2){
  //set the up direction duty cycle
   PWM_Duty(190);
  //Set the motor direction to move elevator DOWN 	
   PTAD = (PTAD & ~(PTAD_PTAD7_MASK))|(PTAD_PTAD6_MASK);
 
  }else {
  //Set the motor direction to stop elevator. 	 
   PTAD = (PTAD & ~(PTAD_PTAD7_MASK|PTAD_PTAD6_MASK));
  }
   if(nextstate != 0)
   currentstate = nextstate;  

}

//*******************************************************
// IRQ Handler: Jumps this ISR when sensor senses a signal
// This scans the sensors and calls the motor controller
// to set the next signals to the motor
// The control will be mostly in this ISR since the elevator
// always will stop in one of the levels. Hence the motor
// controller called from here. 
//*******************************************************
void interrupt 6 IRQHan(void){
asm sei;
scanIRSensor();
motorController();
asm cli;
}

//*******************************************************
// Scan the IR sensor to detect the level
//*******************************************************
void scanIRSensor(void){
  int value;
 
  value = PTAD & 0x1C;
   switch(value){
    case 16: button = 3;         // Assign the level
               LCDString("IR3"); // Used for debugging
               break;
    case 8: button = 2;          // Assign the level
               LCDString("IR2"); // Used for debugging
               break;
    case 4: button = 1;          // Assign the level
               LCDString("IR1"); // Used for debugging
               break;
    case 28:  LCDClear();        // Used to debugging  
              LCDInt(28);        // if IR sensors mismatch
              LCDString("IRErr");
              break;
    case 24:  LCDClear();        // Used to debugging  
              LCDInt(24);        // if IR sensors mismatch
              LCDString("IRErr");
              break;
    case 20:  LCDClear();        // Used to debugging  
              LCDInt(20);        // if IR sensors mismatch
              LCDString("IRErr");
              break;
    case 12:  LCDClear();        // Used to debugging  
              LCDInt(12);        // if IR sensors mismatch
              LCDString("IRErr");
              break;
    default:  break;             //Shouldnt happen
                      
  }
}

//*************************************************************
// IRQ Initialization
//*************************************************************
void IRQ_Init(void){
asm sei // Make atomic
INTCR = 0x40;
asm cli
}

//*************************************************************
// XIRQ Handler 
//*************************************************************
void interrupt 5 XIRQHan(void){
INTCR =0x00;        
LCDString("XIRQ"); // Debug statement
scan();            // scan Keypad
INTCR =0x40;
}

//*************************************************************
//Input and Scan the keyboard
//*************************************************************
void scan(){

 PTT=1;                                   //first row

 scanInput(ReadInput());                  //Read
 PTT=2;                                   //second row

 scanInput(ReadInput());                  //Read
 PTT=4;                                   //Third row

 scanInput(ReadInput());                  //Read
 PTT=8;                                   //Four row

 scanInput(ReadInput());                  //Read
 
 Timer_Wait10ms();                        //Bounce delay 
 PTT=0x0F;                                //Tie 4 inputs to high 
 
}
//*************************************************************
//Convert to corresponding value on keyboard
//and store in global variable.
//*************************************************************
void scanInput(int value) 
{
value = value & 0x7F;  // Take 0 : 6 only, 7th bit discarded
switch(value){

  //1
  case 17: break;                          // Ignored 

  //2
  case 33: break;                          // Ignored

  //3
  case 65: break;                          // Ignored

  //4 :up1
  case 18: up1 = 1;                        // Level 1 (outside elevator) button pressed to go up
           break;

  //5 :up2
  case 34: up2 = 1;                        // Level 2 (outside elevator) button pressed to go up
           break;

  //6 :down2
  case 66: down2 = 1;                      // Level 2 (outside elevator) button pressed to go down
           break;

  //7 : level 1
  case 20: level1 = 1;                     // Level 1 (inside elevator) button pressed to go to level 1
           break;

  //8  : level 2			   
  case 36: level2 = 1;                     // Level 2 (inside elevator) button pressed to go to level 2
           break;

  //9  : level 3                           
  case 68: level3 = 1;                     // Level 3 (inside elevator) button pressed to go to level 3
           break;

  //0   : down 3
  case 40: down3 = 1;                      // Level 3 (outside elevator) button pressed to go down
           break;

  //default
  default: break;                          //Should not happen
  }
 }


//***************************************************************************
//Intialising ports Port T, E, P and AD
//***************************************************************************

void Init(void)
{
 
  //Set the data direction of bits 0-7 on port T to inputs
  DDRT = 0x0F;
  PPST = 0xFF;       // set to pull down port T bits
  PERT = 0x70;       // enable the pull down input bits.
 
  //Set the data direction of bits 4-7 on port AD to input.
  DDRAD = 0xE0;
  //Setting ATDDIEN to xFF to make port AD general I/O port
  ATDDIEN = 0xFF;
  //Setting port E and P to input.
  DDRE = 0x00;
  DDRP = 0x00;
}

//**************************************************************************
//Read Input from Dipswitch 
//**************************************************************************
int ReadInput(void)
{
//Assebly code clear reg A and B and then loads them with Port T value
  asm{
  CLRA
  CLRB
  LDAB PTT
  }

}

//****************************************************************************
// All the code below are for LCD, reused from the previous lab assignment
//****************************************************************************
// See document AN1774.pdf pg 11
// Purpose:  These set of instructions initialize the LCD screen
// after power ON.  The necessary 4bit data mode is set
// and requires two writes for each write.
//****************************************************************************
void LCDInit() {
  //set up SPI to write to LCD
  SPICR1 = 0x5E;
  //Data Sheet - PG419
  //bit7 - SPI Interrupt Enable Bit
  //bit6 - SPI System Enable Bit
  //bit5 - SPI Transmit Interrupt Enable
  //bit4 - SPI Master/Slave Mode Select Bit
  //bit3 - SPI Clock Polarity Bit
  //bit2 - SPI Clock Phase Bit
  //bit1 - Slave Select Output Enable
  //bit0 - LSB-First Enable
 
 
 
  SPICR2 = 0x10;
  //bit 4 - Mode Fault Enable Bit
  //bit 3 - Output Enable in the Bidirectional Mode of Operation
  //bit 1 - SPI Stop in LCDdelay Mode Bit
  //bit 0 - Serial Pin Control Bit 0
 
 
  //MISO - Master In, Serial Out
  //MOSI - Master Out, Serial In
 
 
 
 
  //baud rate = 8 MHz / 640 = 12.5 KHz
  //0100 0110
  //baud rate = 8 MHz / 8 = 1 MHz
  //0111 0000
  SPIBR = 0x70;
 
  LCDdelay(10);
 
 
  LCDWR(0x03);  // Set interface is 8bits
  LCDdelay(10);
  LCDWR(0x03);  // Set interface is 8 bits
  LCDdelay(10);
  LCDWR(0x03);  // Set interface is 8 bits
  LCDdelay(1);
  LCDWR(0x02);  // Set interface is 4 bits
 
  LCDWR(0x02);  // Set interface is 4 bits
  LCDWR(0x08);  // Specify Display Lines and Fonts
  LCDdelay(10);            // 1 = # of lines, 0 Font
 
  //Display OFF
  LCDWR(0x00);
  LCDWR(0x08);
  LCDdelay(10);
 
  //Clear display
  LCDWR(0x00);
  LCDWR(0x01);
  LCDdelay(16);
 
  //Entry mode Set
  LCDWR(0x00);
  LCDWR(0x06);
  LCDdelay(10);
 
 
  //Turn display on with blinking cursor
  LCDWR(0x00);
  LCDWR(0x0F);

}

//******************************************************************************
//Purpose:  This function clears the data from the LCD screen and returns
//          the cursor back to home.
//******************************************************************************
void LCDClear() {
 //Clear
  LCDWR(0x00);
  LCDWR(0x01);
  LCDdelay(10);

  //Return the cursor home
  LCDWR(0x00);
  LCDWR(0x02);
  LCDdelay(10);

}




//******************************************************************************
//Purpose:  LCDCursorOff turns off the cursor indicator of the LCD display
//          proceeds to send the new data.
//******************************************************************************
void LCDCursorOff(){
  LCDWR(0x00);
  LCDWR(0x0C);
}


//******************************************************************************
//Purpose:  LCDCursorOn turns on the cursor indicator of the LCD display
//     
//******************************************************************************
void LCDCursorOn(){
  LCDWR(0x00);
  LCDWR(0x0F);
}



//******************************************************************************
//Purpose:  spiWR waits for the SPI to report it's ready to accept new data and then
//          proceeds to send the new data.
//******************************************************************************
void spiWR(unsigned char data) {

  while (!(SPISR & 0x20));                              //Loop until SPTEF = 1
  SPIDR = data;                                         //Send data out to 74HC595 Chip
}





//******************************************************************************
//Purpose:  LCDString accepts a string and sends each character
//          to the LCDChar function to output the character onto the LCD screen.
//******************************************************************************
void LCDString(char *pt){
  int temp = 0;
  int j = 0;
  while(*pt) {
    if(*pt == '\n') {
      for(j = 0; j < (35-temp); j++) {
        LCDChar(' ');
      }
    pt++;
    temp=0;
    }
    else {
    LCDChar(*pt);
    pt++;
    temp++;
    }
  }
}

//******************************************************************************
//Purpose:  LCDChar sends the proper communication over the SPI to the
//          74HC95 chip.  The chip in turn communicates with the LCD module as
//          specified in the AN1774 document.
//******************************************************************************                                                                       
void LCDChar(unsigned char outchar){

  // Output the higher four bits.
  spiWR((0x0F&(outchar>>4)) & ~ENABLE_BIT | RS_BIT);    // Place data onto bus
  LCDdelay(1);
  spiWR((0x0F&(outchar>>4)) | ENABLE_BIT | RS_BIT);     // Set EN
  LCDdelay(1);                                          // LCDdelay for 1 ms.
  spiWR((0x0F&(outchar>>4)) & ~ENABLE_BIT | RS_BIT);    // Clear EN.
  LCDdelay(1);
 
  // Output the lower four bits.
  spiWR((0x0F&(outchar)) & ~ENABLE_BIT | RS_BIT);       // Place lower four bits onto bus.
  LCDdelay(1);
  spiWR((0x0F&(outchar)) | ENABLE_BIT | RS_BIT);        // Set EN
  LCDdelay(1);                                          // LCDdelay for 1ms.
  spiWR((0x0F&(outchar)) & ~ENABLE_BIT | RS_BIT);       // Clear EN
  LCDdelay(1);
 
}

//******************************************************************************
//Purpose:  Displays a character from 1 to 9     
//******************************************************************************                                                                       
void LCDNum(int val)
{
  //Add the offset in the ascii table
  //to get the character code
  unsigned char outchar=48+val;
 
  //print out the new character
  LCDChar(outchar);

}

//******************************************************************************
//Purpose:  Displays number from 1 to 128.  
//******************************************************************************
void LCDDecimal(unsigned char val)
{
  unsigned char low;
  unsigned char med;
  unsigned char high;
 
  //Get the lowest digit
  low=val%10;

  //shift right in decimal
  val=val/10;
 
  //Get the second digit
  med=val%10;
;
  //shift right in decimal
  val=val/10;
 
  //get the high digit
  high=val%10;
 
  //Print the digits from high to low (left to right)
  LCDNum(high);
  LCDNum(med);
  LCDNum(low);
}

//******************************************************************************
//Purpose:  Displays a 5 digit integer..  
//******************************************************************************
void LCDInt(unsigned int val)
{
   unsigned char part0;
   unsigned char part1;
   unsigned char part2;
   unsigned char part3;
   unsigned char part4;
 
  //get the first digit
  part0=val%10;

//shift right in decimal
  val=val/10;
 
  //get the second digit
  part1=val%10;

//shift right in decimal
  val=val/10;
 
  //get the third digit
  part2=val%10;

//shift right in decimal
  val=val/10;
 
  //get the fourth digit
  part3=val%10;

//shift right in decimal
  val=val/10;
 
  //get the fifth digit
  part4=val%10;
 
  //Print the digits from high to low (left to right)
  LCDNum(part4);
  LCDNum(part3);
  LCDNum(part2);
  LCDNum(part1);
  LCDNum(part0);
}

//******************************************************************************
//Purpose:  Displays two Hex digits preceded by "0x"..  
//******************************************************************************
void LCDHex(unsigned char val)
{
  unsigned char upper;
  unsigned char lower;
 
  //Print the hex symbol
  LCDString("0x");
 
  //put the high 4 bits in upper
  upper=val>>4;
  upper=upper&0x0F;
 
  //put the low 4 bits in lower
  lower=val&0x0F;
 
  //Print upper
  if(upper < 10)
  {
     //if less than 10 it is a digit
     LCDNum(upper);
  }
  else
  {
     //if more than 10 it is a letter
     //Add the ascii offset-10 of the letters
     upper=upper+55;
     LCDChar(upper);
  }
 
  //Print lower
  if(lower < 10)
  {
     //if less than 10 it is a digit
     LCDNum(lower);
  }
  else
  {
     //if more than 10 it is a letter
     //Add the ascii offset-10 of the letters
     lower=lower+55;
     LCDChar(lower);
  }
 
}

//******************************************************************************
//Purpose:  LCDWR sends a 4-bit messages to the LCD module.  Used to send
//          setup instructions to the LCD module.
//******************************************************************************

void LCDWR(unsigned char data) {
  spiWR(data & ~ENABLE_BIT);
  LCDdelay(1);
  spiWR(data | ENABLE_BIT);     // Set EN
  LCDdelay(1);
  spiWR(data & ~ENABLE_BIT);    // Clear EN
  LCDdelay(1);
}

//******************************************************************************
//Purpose:  LCDdelay is a custom delay function that loops for a number of cycles
//          based on the number of miliseconds specified in its parameter.
//******************************************************************************
void LCDdelay(unsigned long ms) {
char i;

for (i=0;i < ms; i++) {
 
asm {
PSHX
LDX #$640
Loop:
NOP
NOP
DBNE X, Loop
PULX
}

}
}