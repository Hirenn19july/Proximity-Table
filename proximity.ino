

/***                   
PIN ASSIGNMENTS ON ATMEGA328

PC6 (PCINT14/RESET)            
PC5 (ADC5/SCL/PCINT13)            // I2C Clock input
PC4 (ADC4/SDA/PCINT12)            // I2C Data input
PC3 (ADC3/PCINT11)                    //Sensor 4 IR Receiver
PC2 (ADC2/PCINT10)                    //Sensor 3 IR Receiver
PC1 (ADC1/PCINT9)                      //Sensor 2 IR Receiver
PC0 (ADC0/PCINT8)                      //Sensor 1 IR Receiver

PB7 (PCINT7/XTAL2/TOSC2)         //IR 4 Trigger...extra
PB6 (PCINT6/XTAL1/TOSC1)         //IR 3 Trigger...extra
PB5 (SCK/PCINT5)                           //IR 2 Trigger
PB4 (MISO/PCINT4)                        //IR 1 Trigger
PB3 (MOSI/OC2A/PCINT3)            //PWM 3
PB2 (SS/OC1B/PCINT2)           
PB1 (OC1A/PCINT1)              //IR 4 Trigger 
PB0 (PCINT0/CLKO/ICP1)     //IR 3 Trigger      

PD0 (PCINT16/RXD)                
PD1 (PCINT17/TXD)                
PD2 (PCINT18/INT0)                
PD3 (PCINT19/OC2B/INT1)         //PWM 4
PD4 (PCINT20/XCK/T0)            
PD5 (PCINT21/OC0B/T1)             //PWM 2
PD6 (PCINT22/OC0A/AIN0)         //PWM 1
PD7 (PCINT23/AIN1)             
***/
#define IR_1_ON PORTB |= (1<<4)
#define IR_2_ON PORTB |= (1<<5)
#define IR_3_ON PORTB |= (1<<0)
#define IR_4_ON PORTB |= (1<<1)
#define IR_1_OFF PORTB &= ~(1<<4)
#define IR_2_OFF PORTB &= ~(1<<5)
#define IR_3_OFF PORTB &= ~(1<<0)
#define IR_4_OFF PORTB &= ~(1<<1)

#define PWM1 6                    //PORTD        PWM pin assignments
#define PWM2 5                    //PORTD
#define PWM3 3                    //PORTB
#define PWM4 3                    //PORTD
/****Function Declarations****/
int ADC_read(int Temp); 
void A2D_Channel_Select(unsigned char channel);
void Init_ADC(void);
void Init_Timer0(void);
void Init_Timer1(void);
void Init_Timer2(void);
void Delay(void);
void Calibrate_Sensors(void);
//void Init_I2C_Slave_Rx(void);
/****Global Variable Declarations****/
volatile int Sensor_Values_Updated = LOW;                   
volatile char Timer1_Overflow = 0;
volatile unsigned char channel = 0;
volatile int Amb_Sensor_1 = 0, Amb_Sensor_2 = 0, Amb_Sensor_3 = 0, Amb_Sensor_4 = 0;
volatile int Sensor_1 = 0, Sensor_2 = 0, Sensor_3 = 0, Sensor_4 = 0;
volatile int Initial_1 = 0, Initial_2 = 0, Initial_3 = 0, Initial_4 = 0;
volatile int New_PWM1 = 0, New_PWM2 = 0, New_PWM3 = 0, New_PWM4 = 0;               
volatile int Old_PWM1 = 0, Old_PWM2 = 0, Old_PWM3 = 0, Old_PWM4 = 0;

unsigned char buffer = 8;
void setup() {
   // Serial.begin(9600);
    DDRB = 0xff;
    //make sure IR emitters are turned off, and PWM 3
    PORTB &= ~((1 << 1)|(1 << 0)|(1 << 5)|(1 << 4)|(1 << 3));               
    DDRC = 0x00;                    //make PORT C inputs
   
    DDRD = 0xff;
    PORTD = 0x00;                    //set all of PORT D low. ensures 
   // Init_ADC();
    sei();
    Calibrate_Sensors();
   // Serial.println("C");
    PORTD |= (1 << PWM1);            //blink to indicate end of Calibration

   Delay1();
   
   PORTD &= ~(1 << PWM1);

    Init_Timer0();
    Init_Timer2();
}

void loop() {
  

}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
ISR(TIMER1_OVF_vect)  
    {
        Timer1_Overflow++;            //increment timer overflow variable
     //  Serial.println("ISR");
       
        switch(Timer1_Overflow)
            {
                case 1:                   
                   // A2D_Channel_Select(0);                            //select ADC channel 0
                    Amb_Sensor_1 = ADC_read(A0);
                //  Amb_Sensor_1= analogRead(analogInPin);
                   // Serial.println(Amb_Sensor_1);//take ambient IR sensor reading
                    IR_1_ON;                                                        //turn on IR 1 LED, PORTB |= (1<<4)
                    Delay();                                                           //delay for the IR receiver to settle
                    Sensor_1 = ADC_read(A0);                            //take active ADC reading of IR receiver
                   // Serial.println(Sensor_1);
                    IR_1_OFF;                                                       //turn off IR 1 LED   
                    New_PWM1 = (Sensor_1 - Amb_Sensor_1) - Initial_1;    //condition readings
                    if(New_PWM1 <= 0)    { New_PWM1 = 0; }                //prevent negative numbers
                   
              // simple low-pass filter, (87.5% * Old) + (12.5% * New). It just takes the old value and   weights it more than the older value. Has the same effect of slowing down change, which is crucial in providing fluid changes in brightness
                    New_PWM1 = ((7*Old_PWM1)>>3) + (New_PWM1>>3);     
 
                    if(OCR0A >= 1)    {DDRD |= (1 << PWM1);}
                    else { DDRD &= ~(1 << PWM1); }                        //turn off LEDs completely

//artificially increase the value of sensor reading, not entirely necessary, but makes the sensor seem more sensitiveby being brighter sooner
                    New_PWM1 <<= 2;                                  
                    if(New_PWM1 > 255)    { New_PWM1 = 255; }
                    OCR0A = New_PWM1;
                    New_PWM1 >>= 2;
   
                    Old_PWM1 = New_PWM1; //Serial.println(Sensor_Values_Updated);  
                    break;
                    
                case 2:
                  //  A2D_Channel_Select(1);                            //select ADC channel 1
                    Amb_Sensor_2 = ADC_read(A1);
                    IR_2_ON;                                        //turn on IR 2 LED, PORTB |= (1<<5)
                    Delay();                                        //delay for the IR receiver to settle
                    Sensor_2 = ADC_read(A1);                            //take ADC reading
                    IR_2_OFF;                                        //turn off IR 2 LED   

                    New_PWM2 = (Sensor_2 - Amb_Sensor_2) - Initial_2;
                    if(New_PWM2 < 0)    { New_PWM2 = 0; }
                   
                    New_PWM2 = ((7*Old_PWM2)>>3) + (New_PWM2>>3);
                    if(OCR0B >= 1)    {DDRD |= (1 << PWM2);}
                    else { DDRD &= ~(1 << PWM2); }

                    New_PWM2 <<= 2;
                    if(New_PWM2 > 255)    { New_PWM2 = 255; }
                    OCR0B = New_PWM2;
                    New_PWM2 >>= 2;
                
                    Old_PWM2 = New_PWM2; 
                    break;
                   
                case 3:
                   // A2D_Channel_Select(2);                            //select ADC channel 2
                    Amb_Sensor_3 = ADC_read(A2);
                    IR_3_ON;                                        //turn on IR 3 LED, PORTB |= (1<<6)
                    Delay();                                        //delay for the IR receiver to settle
                    Sensor_3 = ADC_read(A2);                            //take ADC reading
                    IR_3_OFF;                                        //turn off IR 3 LED   

                    New_PWM3 = (Sensor_3 - Amb_Sensor_3) - Initial_3;
                    if(New_PWM3 < 0)    { New_PWM3 = 0; }
                   
                    New_PWM3 = ((7*Old_PWM3)>>3) + (New_PWM3>>3);
                    if(OCR2A >= 1)    {DDRB |= (1 << PWM3);}
                    else { DDRB &= ~(1 << PWM3); }
                    New_PWM3 <<= 2;
                    if(New_PWM3 > 255)    { New_PWM3 = 255; }
                    OCR2A = New_PWM3;
                    New_PWM3 >>= 2;
  
                    Old_PWM3 = New_PWM3;  
                    break;
                   
                case 4:
                  //  A2D_Channel_Select(3);                            //select ADC channel 3
                    Amb_Sensor_4 = ADC_read(A3);
                    IR_4_ON;                                        //turn on IR 4 LED, PORTB |= (1<<7)
                    Delay();                                        //delay for the IR receiver to settle
                    Sensor_4 = ADC_read(A3);                            //take ADC reading
                    IR_4_OFF;                                        //turn off IR 4 LED   

                    New_PWM4 = (Sensor_4 - Amb_Sensor_4) - Initial_4;
                    if(New_PWM4 < 0)    { New_PWM4 = 0; }
                   
                    New_PWM4 = ((7*Old_PWM4)>>3) + (New_PWM4>>3);
                    if(OCR2B >= 1)    {DDRD |= (1 << PWM4);}
                    else { DDRD &= ~(1 << PWM4); }
                    New_PWM4 <<= 2;
                    if(New_PWM4 > 255)    { New_PWM4 = 255; }
                    OCR2B = New_PWM4;
                    New_PWM4 >>= 2;
                    
                    Old_PWM4 = New_PWM4;//reset                                       
                    Timer1_Overflow = 0;  Sensor_Values_Updated = HIGH;               //new values ready
                    break;       
            }//end switch
    }//end ISR
 void Calibrate_Sensors(void) //establish initial ambient sensor values
    {
        char q = 0;   
       
        Init_Timer1();
      //  Serial.println("Calibration ok");_delay_ms(600);_delay_ms(600);
        for(q=0; q<32; q++)            //should take one second-ish
            {
                //wait for Sensor cycle to be done, then gather sensors values
             while(Sensor_Values_Updated == LOW)    {}
             // while(Sensor_Values_Updated == 0);
            //  _delay_ms(600);
               
                Initial_1 += (Sensor_1 - Amb_Sensor_1);        //initial difference
                Initial_2 += (Sensor_2 - Amb_Sensor_2);
                Initial_3 += (Sensor_3 - Amb_Sensor_3);
                Initial_4 += (Sensor_4 - Amb_Sensor_4);
                Sensor_Values_Updated = LOW;                //reset
                
            }//end for

//condition Initial Ambient Sensor values, plus a buffer
            Initial_1 = (Initial_1 >> 5) + buffer;          
            Initial_2 = (Initial_2 >> 5) + buffer;
            Initial_3 = (Initial_3 >> 5) + buffer;
            Initial_4 = (Initial_4 >> 5) + buffer;  
    }
    void Init_Timer0(void)                            //PWM for sensors 1 & 2
    {
        //Fast PWM, non-inverting, WGM02-WGM00 == 011, no overflow interrupt
        TCCR0A |= ((1 << COM0A1)|(1 << COM0B1)|(1 << WGM01)|(1 << WGM00));
        TCCR0B |= (1 << CS00);        //start clock, no prescale
    }
    void Init_Timer2(void)                            //PWM for sensors 3 & 4
    {
        //Fast PWM, non-inverting, WGM22-WGM20 == 011, no overflow interrupt
        TCCR2A |= ((1 << COM2A1)|(1 << COM2B1)|(1 << WGM21)|(1 << WGM20));
        TCCR2B |= (1 << CS20);        //start clock, no prescale
    } 
void Init_Timer1(void)
{
        //no PWM, enable overflow interrupt,
        //TOP == 0xFFFF == 65536 cycles == roughly 122 overflow interrupts/sec
        TCCR1B |= (1 << CS10);
        TIMSK1 |= (1 << TOIE1);
}
int ADC_read(int Temp)                        /***select ADC channel prior to calling this function***/
    {       
        int ADC_value = 0;
        int ADCsample;
        char i;  
        for (i=0; i<64; i++)                   
            {
     
                ADCsample =analogRead(Temp); 
                //ADCsample = ADCH;        
                //ADCsample += (ADCH<<8);        //Left shift the top two bits 8 places                       
                ADC_value += ADCsample;            //add ADCsample to ADC_sensor
            }               
//average sample by right shifting 6 places, same as dividing by 64
        ADC_value = (ADC_value >> 6);           
               
        return ADC_value;   
    }
/*void A2D_Channel_Select(unsigned char channel)               
    {
       
        switch (channel)
            {
                case 0:            //select A2D channel 0
                    ADMUX &= ~((1 << 3)|(1 << 2)|(1 << 1)|(1 << 0));
                    break;   
       
                case 1:            //select A2D channel 1
                    ADMUX &= ~((1 << 3)|(1 << 2)|(1 << 1));
                    ADMUX |= (1 << 0);
                    break;
               
                case 2:            //select A2D channel 2
                    ADMUX &= ~((1 << 3)|(1 << 2)|(1 << 0));
                    ADMUX |= (1 << 1);
                    break;   
       
                case 3:            //select A2D channel 3
                    ADMUX &= ~((1 << 3)|(1 << 2));
                    ADMUX |= ((1 << 1)|(1 << 0));
                    break;
                 
            }//end switch
    }*/
   
void Delay(void)
{
        _delay_us(200);
}
void Delay1(void)
{
        _delay_us(10);
}
