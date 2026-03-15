//
//  TWI_Slave.c
//  TWI_Slave
//
//  Created by Sysadmin on 14.10.07.
//  Copyright __MyCompanyName__ 2007. All rights reserved.
//

#include <Arduino.h> 

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
//#include <avr/sleep.h>
#include <avr/eeprom.h>
#include <inttypes.h>
#include <avr/wdt.h>
#include "defines.h"
#include <stdint.h>
#include <util/twi.h>

#include <Wire.h>
#include <U8g2lib.h>

U8G2_SSD1306_128X32_UNIVISION_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);


#include "lcd.c"

#include "display.h"

#include "text.h"

/*
*******************************************
*******************************************
BOD-Level auf 2.7V eingestellt. Efuse 0xFD 
*******************************************
*******************************************


*/

#define SHOWDISPLAY 0


//#include "adc.c"
#define TW_STATUS   (TWSR & TW_STATUS_MASK)



//***********************************
						
uint8_t   LOK_ADRESSE = 0x0F;    //   11001100	Trinär mit Adresse 20 20 * DIP 1100

//uint8_t LOK_ADRESSE = 0xBF; // Weiche 


//	Trinaer-Adressen								
//uint8_t  LOK_ADRESSE = 0x80;     //	1000 0000	Trinär mit Adresse 12 00 
//uint8_t  LOK_ADRESSE = 0xB0;     //	1011 0000	Trinär mit Adresse 12 22


//***********************************

/*
 commands
 LO     0x0202  // 0000001000000010
 OPEN   0x02FE  // 0000001011111110
 HI     0xFEFE  // 1111111011111110
 */


//#define OUTPORT	PORTD		// Ausgang fuer Motor

//#define INPORT   PORTD  // Input signal auf INT0
#define INPIN   PIND  // Input signal

#define DATAPIN  2 


#define LOOPLEDPORT     PORTB
#define LOOPLEDDDR      DDRB
#define LOOPLED         0 // 

#define INT0_RISING	   0
#define INT0_FALLING		1

volatile uint8_t   loopstatus=0x00;            

void lcd_puts(const unsigned char *s);

// EADOGM
volatile uint16_t laufsekunde=0;
volatile uint8_t laufminute=0;
volatile uint8_t laufstunde=0;
volatile uint16_t motorsekunde=0;
volatile uint16_t motorminute=0;
volatile uint16_t stopsekunde=0;
volatile uint16_t stopminute=0;
volatile uint16_t batteriespannung =0;
volatile uint16_t updatecounter =0;
volatile uint8_t                 curr_screen=0; // aktueller screen
volatile uint8_t                 curr_cursorzeile=0; // aktuelle zeile des cursors
volatile uint8_t                 curr_cursorspalte=0; // aktuelle colonne des cursors
volatile uint8_t                 last_cursorzeile=0; // letzte zeile des cursors
volatile uint8_t                 last_cursorspalte=0; // letzte colonne des cursors
volatile uint16_t                posregister[8][8]={}; // Aktueller screen: werte fuer page und daraufliegende col fuer Menueintraege (hex). geladen aus progmem
volatile uint16_t                cursorpos[8][8]={}; // Aktueller screen: werte fuer page und darauf liegende col fuer den cursor
volatile uint16_t                 blink_cursorpos=0xFFFF;
volatile uint8_t                 displaystatus = 0; // bits for sending display data
                                          // 0-3: sendposition
                                          // 4,5,6: 
                                          // 7: aktivbit
volatile uint8_t                 displaydata[8]  = {};

//
volatile uint8_t	   INT0status=0x00;				
volatile uint8_t	   signalstatus=0x00; // status TRIT
volatile uint8_t     pausestatus=0x00;

volatile uint8_t     richtungstatus=0x00;


volatile uint8_t     address=0x00; 
volatile uint8_t     data=0x00;   

volatile uint16_t    saveEEPROM_Addresse = 0;
volatile uint8_t     EEPROM_savestatus=0x00;   
volatile uint8_t     EEPROM_lastsavedstatus = 0;



volatile uint8_t	HIimpulsdauerPuffer=22;		//	Puffer fuer HIimpulsdauer
volatile uint8_t	HIimpulsdauerSpeicher=0;		//	Speicher  fuer HIimpulsdauer

volatile uint8_t   LOimpulsdauerOK=0;   

volatile uint8_t   pausecounter = 0; //  neue daten detektieren
volatile uint8_t   abstandcounter = 0; // zweites Paket detektieren
volatile uint8_t   paketcounter = 0; // 

volatile uint8_t   tritposition = 0; // nummer des trit im Paket
volatile uint8_t   lokadresse = 0;

volatile uint8_t   lokadresseA = 0;
volatile uint8_t   lokadresseB = 0;

volatile uint8_t   lokadresseTRIT = 0; // Adresse mit trit

volatile uint8_t   deflokadresse = 0;
volatile uint8_t   lokstatus=0x00; // Funktion, Richtung

volatile uint8_t   oldlokdata = 0;
volatile uint8_t   lokdata = 0;
volatile uint8_t   deflokdata = 0;

volatile uint8_t   rawdataA = 0;
volatile uint8_t   rawdataB = 0;

// ***
volatile uint8_t   rawfunktionA = 0;
volatile uint8_t   rawfunktionB = 0;

volatile uint8_t     oldspeed = 0;
volatile uint8_t     newspeed = 0;
volatile uint8_t     startspeed = 0; // Anlaufimpuls
volatile uint8_t     speedcode = 0;
volatile int8_t      speedintervall = 0;

volatile uint8_t   ledstatus=0; // status LED

volatile uint8_t   ledonpin = LAMPEA_PIN; // Stirnlampe ON
volatile uint8_t   ledoffpin = LAMPEB_PIN; // Stirnlampe OFF

// ***
volatile uint8_t   speed = 0;

volatile uint8_t   oldfunktion = 0;
volatile uint8_t   funktion = 0;
volatile uint8_t   deffunktion = 0;
volatile uint8_t   waitcounter = 0;
volatile uint8_t   richtungcounter = 0; // delay fuer Richtungsimpuls

volatile uint8_t     pwmpin = MOTORA_PIN;           // Motor PWM
volatile uint8_t     richtungpin = MOTORB_PIN;      // Motor Richtung

volatile uint16_t   motorPWM=0;

volatile uint8_t    speedlookup[15] = {};
uint8_t             speedlookuptable[10][15] =
{
   {0,18,36,54,72,90,108,126,144,162,180,198,216,234,252},  // 0
   {0,30,40,50,60,70,80,90,100,110,120,130,140,150,160},    // 1
   {0,10,20,30,40,50,60,70,80,90,100,110,120,130,140},      // 2
   {0,7,14,21,28,35,42,50,57,64,71,78,85,92,100},           // 3
   {0,33,37,40,44,47,51,55,58,62,65,69,72,76,80},           // 4
   
   {0,41,42,44,47,51,56,61,67,74,82,90,99,109,120},         // 5
   {0,41,43,45,49,54,60,66,74,82,92,103,114,127,140},       // 6
   
   {0,50,65,70,77,90,105,122,140,159,170,188,200,210,220},  // 7
   
   {0,15,20,27,35,44,55,67,80,116,134,153,173,196,220},    // 8
   {0,42,45,51,58,68,79,93,108,125,144,165,188,213,240}     // 9

};

volatile uint8_t     speedindex = 8;
volatile uint8_t     maxspeed =  0; //speedlookuptable[speedindex][14];
volatile uint8_t     minspeed =  0;

volatile uint8_t     lastDIR =  0;
uint8_t              loopledtakt = 0x40;
uint8_t              refreshtakt = 0x50;
uint16_t             speedchangetakt = 0x150; // takt fuer beschleunigen/bremsen

uint16_t             lasteepromaddress = MAX_EEPROM - 1; // letzte benutzte Adresse, max je nach typ
uint8_t              lasteepromdata = 0;

volatile uint8_t loktyptable[4];

uint16_t displaycounter0;
uint16_t displaycounter1;

uint16_t displayfenstercounter = 0; // counter fuer abgelaufene Zeit im Display-Fenster

//extern uint8_t display_init(void);//
void displayfensterfunction(void);

void spi_init(void) // SPI-Pins aktivieren
{
   // https://embedds.com/serial-peripheral-interface-in-avr-microcontrollers/
   //set MOSI, SCK and SS as output
   DDRB |= (1<<PB3)|(1<<PB5)|(1<<PB2);
   //set SS to high
   PORTB |= (1<<PB2);
   //enable master SPI at clock rate Fck/2
   SPCR = (1<<SPE)|(1<<MSTR);
   SPSR |= (1<<SPI2X);
}

void slaveinit(void)
{
   OSZIDDR |= (1<<OSZI_PULS_A);
 	OSZIPORT |= (1<<OSZI_PULS_A);	//Pin 6 von PORT D als Ausgang fuer OSZI A
		//Pin 7 von PORT D als Ausgang fuer SOSZI B
   
   OSZIPORT |= (1<<OSZI_PULS_B);   //Pin 6 von PORT D als Ausgang fuer OSZI A
   OSZIDDR |= (1<<OSZI_PULS_B);   //Pin 7 von PORT D als Ausgang fuer SOSZI B
   //OSZIPORT |= (1<<SYNC);   //Pin 6 von PORT D als Ausgang fuer OSZI A
   //OSZIDDR |= (1<<SYNC);   //Pin 7 von PORT D als Ausgang fuer SOSZI B

   //OSZIPORT |= (1<<INT_0);   //
   //OSZIDDR |= (1<<INT_0);   //Pin 7 von PORT D als Ausgang fuer SOSZI B
 
   displaydata[0] = 3;
   displaydata[1] = 44;
   
	//LCD
	LCD_DDR |= (1<<LCD_RSDS_PIN);	//Pin 5 von PORT B als Ausgang fuer LCD
 	LCD_DDR |= (1<<LCD_ENABLE_PIN);	//Pin 6 von PORT B als Ausgang fuer LCD
	LCD_DDR |= (1<<LCD_CLOCK_PIN);	//Pin 7 von PORT B als Ausgang fuer LCD

   TESTDDR |= (1<<TEST0); // test0
   TESTPORT |= (1<<TEST0); // HI
   TESTDDR &= ~(1<<TEST1); // test1 INPUT
   TESTPORT |= (1<<TEST1); // HI
   
   MOTORDDR |= (1<<MOTORA_PIN);  // Output Motor A 
   MOTORPORT |= (1<<MOTORA_PIN); // HI
   
   MOTORDDR |= (1<<MOTORB_PIN);  // Output Motor B 
   MOTORPORT |= (1<<MOTORB_PIN); // HI
   
   LAMPEDDR |= (1<<LAMPEA_PIN);  // Lampe A
   LAMPEPORT &= ~(1<<LAMPEA_PIN); // LO
    
   LAMPEDDR |= (1<<LAMPEB_PIN);  // Lampe B
   LAMPEPORT &= ~(1<<LAMPEB_PIN); // LO


   

   if (SHOWDISPLAY == 1)
   {
      spi_init();
      _delay_ms(5);
      uint8_t erfolg = display_init();
      _delay_ms(5);
   }
   
}


void timer2 (uint8_t wert) 
{ 
   //	TCCR2 |= (1<<CS02);				//8-Bit Timer, Timer clock = system clock/256

   //Takt fuer Servo
   //	TCCR2 |= (1<<CS20)|(1<<CS21);	//Takt /64	Intervall 64 us

	TCCR2A |= (1<<WGM21);		//	ClearTimerOnCompareMatch CTC
   TCCR2B |= (1<<CS21); // no prescaler
	//OC2 takt
	TIMSK2 |= (1<<OCIE2A);			//CTC Interrupt aktivieren
   TIMSK2 |=(1<<TOIE2);        //interrupt on Compare Match A
	TCNT2 = 0x00;					//Zaehler zuruecksetzen
	
	OCR2A = wert;					//Setzen des Compare Registers auf HI-impulsdauer
} 



void int0_init(void)
{
   EICRA |= (1 << ISC00) | (1 << ISC01);  // Trigger interrupt on rising edge
   EIMSK |= (1 << INT0);  // Enable external interrupt INT0

 //  INT0status |= (1<<INT0_RISING);
   INT0status = 0;
   //INT0status |= (1<<INT0_WAIT);
}

// MARK:  INT0
ISR(INT0_vect) 
{
   if (INT0status == 0) // neue Daten beginnen
   {
      //displaystatus &= ~(1<<DISPLAY_GO); // displayfenster end
      //SYNC_HI();
      //OSZI_A_LO(); 
      INT0status |= (1<<INT0_START);
      INT0status |= (1<<INT0_WAIT); // delay, um Wert des Eingangs zum richtigen Zeitpunkt zu messen
      
      INT0status |= (1<<INT0_PAKET_A); // erstes Paket lesen
      
      pausecounter = 0; // pausen detektieren, reset fuer jedes HI
      abstandcounter = 0;// zweites Paket detektieren, 
      
      waitcounter = 0;
      tritposition = 0;
      funktion = 0;         
   } 

   else // Data im Gang, neuer Interrupt
   {
      INT0status |= (1<<INT0_WAIT);
      pausecounter = 0;
      abstandcounter = 0; 
      waitcounter = 0;
   }
}


ISR(TIMER2_COMPA_vect) // // Schaltet Impuls an MOTOROUT LO wenn speed
{ 
   //OSZI_B_LO();
   if (speed)
   {
      // OSZI_B_LO();
      motorPWM++;
   }
   
   if ((motorPWM > speed) || (speed == 0)) // Impulszeit abgelaufen oder speed ist 0
   {
      MOTORPORT |= (1<<pwmpin);  // Motor OFF    
   }
   
   if (motorPWM >= 250) //ON, neuer Motorimpuls
   {
      MOTORPORT &= ~(1<<pwmpin);
      motorPWM = 0;
   }
   
   if (displayfenstercounter > 8)
   {
      //displayfensterfunktion();
      //displayfenstercounter=0;
   }
   
   // MARK: TIMER2  TIMER2_COMPA INT0
   if (INT0status & (1<<INT0_WAIT))
   {
      waitcounter++; 
      if (waitcounter >2)// Impulsdauer > minimum, nach einer gewissen Zeit den Status abfragen
      {
         //OSZI_A_LO();
         
         INT0status &= ~(1<<INT0_WAIT);
         if (INT0status & (1<<INT0_PAKET_A))
         {
           if (tritposition < 8) // Adresse)
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseA |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseA &= ~(1<<tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // Funktion
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionA |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawfunktionA &= ~(1<<(tritposition-8)); // bit ist 0
               }
            }
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataA |= (1<<((tritposition-10))); // bit ist 1
               }
               else // 
               {
                  rawdataA &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
         }
         //OSZI_A_HI(); 
         if (INT0status & (1<<INT0_PAKET_B))
         {
            //OSZI_A_LO(); 
           if (tritposition < 8) // Adresse)
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  lokadresseB |= (1<<tritposition); // bit ist 1
               }
               else // 
               {
                  lokadresseB &= ~(1<<tritposition); // bit ist 0
               }
            }
            else if (tritposition < 10) // bit 8,9: funktion
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawfunktionB |= (1<<(tritposition-8)); // bit ist 1
               }
               else // 
               {
                  rawfunktionB &= ~(1<<(tritposition-8)); // bit ist 0
               }
            }
            
            else
            {
               if (INPIN & (1<<DATAPIN)) // Pin HI, 
               {
                  rawdataB |= (1<<(tritposition-10)); // bit ist 1
               }
               else 
               {
                  rawdataB &= ~(1<<(tritposition-10)); // bit ist 0
               }
            }
            //OSZI_A_HI();
         }
         //OSZI_A_HI();
         /*
          // Paket anzeigen
          if (INT0status & (1<<INT0_PAKET_B))
          {
          //           TESTPORT |= (1<<TEST2);
          }
          if (INT0status & (1<<INT0_PAKET_A))
          {
          //           TESTPORT |= (1<<TEST1);
          }
          */
         
         if (tritposition < 17)
         {
            tritposition ++;
         }
         else // Paket gelesen
         {
            //OSZI_A_LO();
            // Paket A?
            if (INT0status & (1<<INT0_PAKET_A)) // erstes Paket, Werte speichern
            {
               oldfunktion = funktion;
               
               INT0status &= ~(1<<INT0_PAKET_A); // Bit fuer erstes Paket weg
               INT0status |= (1<<INT0_PAKET_B); // Bit fuer zweites Paket setzen
               tritposition = 0;
            }
            else if (INT0status & (1<<INT0_PAKET_B)) // zweites Paket, Werte testen
            {
               //SYNC_LO();
               displaystatus |= (1<<DISPLAY_GO);
               // // Displayfenster begin
               if(displayfenstercounter++ > 4)
               {
                  displaystatus |= (1<<DISPLAY_GO);
                  displayfenstercounter=0;
               }
               //displayfenstercounter = MAXFENSTERCOUNT;
               
               if (lokadresseA && ((rawfunktionA == rawfunktionB) && (rawdataA == rawdataB) && (lokadresseA == lokadresseB))) // Lokadresse > 0 und Lokadresse und Data OK
               {
                  if (lokadresseB == LOK_ADRESSE)
                  {   
                     OSZI_A_LO();
                     // Daten uebernehmen
                     
                     lokstatus |= (1<<ADDRESSBIT);
                     deflokadresse = lokadresseB;
                     deffunktion = rawfunktionB;        
                     if (deffunktion)
                     {
                        lokstatus |= (1<<FUNKTIONBIT);
                        ledstatus |= (1<<LED_CHANGEBIT); // change setzen   
                     }
                     else
                     {
                        lokstatus &= ~(1<<FUNKTIONBIT);
                        ledstatus |= (1<<LED_CHANGEBIT); // led-change setzen                  
                     }
                     // deflokdata aufbauen
                     for (uint8_t i=0;i<8;i++)
                     {
                        //if ((rawdataB & (1<<(2+i))))
                        if ((rawdataB & (1<<i)))
                        {
                           deflokdata |= (1<<i);
                        }
                        else 
                        {
                           deflokdata &= ~(1<<i);
                        }
                     }
                        // MARK: RICHTUNG 
                     // Richtung
                     if (deflokdata == 0x03) // Wert 1, > Richtung togglen
                     {
                       if (richtungstatus == 0) // letzter Richtungswechel ist abgeschlossen
                        
                        {
                           richtungstatus |= (1<<RICHTUNGCHANGESTARTBIT); // Vorgang starten, speed auf 0 setzen
                        
                           lokstatus |= (1<<LOK_CHANGEBIT); // lok-change setzen
                           ledstatus |= (1<<LED_CHANGEBIT); // led-change setzen

                        } // if !(richtungstatus & (1<<RICHTUNGCHANGESTARTBIT)
                        
                     } // deflokdata == 0x03
                     else // speed anpassen
                     {  
                        if(richtungstatus &(1<<RICHTUNGCHANGEOKBIT)) // richtungswechsel ist erfolgt
                        {
                           richtungstatus = 0;
                        }

                        

                        switch (deflokdata)
                        {
                           case 0:
                              
                              speedcode = 0;
                              lokstatus &= ~(1<<STARTBIT);
                              lokstatus &= ~(1<<RUNBIT); 
                              break;
                           case 0x0C:
                              speedcode = 1;
                              break;
                           case 0x0F:                             
                              speedcode = 2;
                              break;
                           case 0x30:
                              speedcode = 3;
                              break;
                           case 0x33:
                              speedcode = 4;
                              break;
                           case 0x3C:
                              speedcode = 5;
                              break;
                           case 0x3F:
                              speedcode = 6;
                              break;
                           case 0xC0:
                              speedcode = 7;
                              break;
                           case 0xC3:
                              speedcode = 8;
                              break;
                           case 0xCC:
                              speedcode = 9;
                              break;
                           case 0xCF:
                              speedcode = 10;
                              break;
                           case 0xF0:
                              speedcode = 11;
                              break;
                           case 0xF3:
                              speedcode = 12;
                              break;
                           case 0xFC:
                              speedcode = 13;
                              break;
                           case 0xFF:
                              speedcode = 14;
                              break;
                           default:
                              speedcode = 0;
                              break;
                              
                        } // switch (deflokdata)                        
                        //OSZI_B_HI();

                        // ******* speedcode  

                        oldspeed = speed; // Istwert, behalten
                        newspeed = speedlookup[speedcode]; // solllwert
                        
                        // Startbedingung, langsam anfahren bis speedlookup[1]
                        if((speedcode == 1) && !(lokstatus & (1<<STARTBIT))  && !(lokstatus & (1<<RUNBIT))) // Start, noch nicht gesetzt  
                           {
                              startspeed = speedlookup[STARTINDEX] ;
                              oldspeed = speedlookup[1] / 2;
                              newspeed = speedlookup[1]; //
                              lokstatus |= (1<<STARTBIT);
                           }
                        else
                           {
                              newspeed = speedlookup[speedcode]; // zielwert
                           }

                        if(speedcode > 0)
                        {
                           lokstatus |= (1<<RUNBIT); // lok in bewegung
                        }
                        else
                        {
                           lokstatus &= ~(1<<RUNBIT); // lok steht still
                        }

                        speedintervall = (newspeed - oldspeed)>>2; // 4 teile                          
                           if((speedcode > 2) && (speedintervall > 4) )
                           {
                              speedintervall = 4;
                           }
                     }
                     //SYNC_HI();
                     OSZI_A_HI();
                  }
                  else 
                  {
                     // aussteigen
                     //deflokadresse = 0;
                     //deflokdata = 0xCA;
                     INT0status = 0;
                     return;
                  }
                  
               }// if (lokadresseA &&...
               else 
               {
                  lokstatus &= ~(1<<ADDRESSBIT);
                  // aussteigen
                  //deflokdata = 0xCA;
                  INT0status = 0;
                  return;
               }
               
               INT0status |= (1<<INT0_END);
               if (INT0status & (1<<INT0_PAKET_B))
               {
                  //               TESTPORT |= (1<<TEST2);
               }
               //SYNC_HI();
               //OSZI_A_HI();
            } // End Paket B         
         }
      } // waitcounter > 2
   } // if INT0_WAIT
   
   if (INPIN & (1<<DATAPIN)) // Pin HI, input   im Gang
   {
      //      HIimpulsdauer++; // zaehlen
   }
   else  // LO, input fertig, Bilanz
   {
      if (abstandcounter < 20)
      {
         abstandcounter++;
      }
      else //if (abstandcounter ) // Paket 2
      {
         abstandcounter = 0;         
      }
      
      if (pausecounter < 120)
      {
         pausecounter ++; // pausencounter incrementieren
      }
      else 
      {
         //OSZIBHI; //pause detektiert
         pausecounter = 0;
         INT0status = 0; //Neue Daten abwarten
         paketcounter++;
        return;
      }
      
   } // input LO
   //OSZI_B_HI();pwmpin = MOTORA_PIN;
} // ISR(TIMER2_COMPA_vect)

void displayfensterfunction(void)
{
   displayfenstercounter = 0;
   _delay_ms(2);
}

// Funktion, um ein Byte in den EEPROM zu schreiben
void EEPROM_Write(uint16_t address, uint8_t data) {
    eeprom_update_byte((uint8_t*)address, data);
}

// Funktion, um ein Byte aus dem EEPROM zu lesen
uint8_t EEPROM_Read(uint16_t address) {
    return eeprom_read_byte((uint8_t*)address);
}

void EEPROM_Clear(void) 
{
   uint16_t addr = 0;
   while (addr++ < MAX_EEPROM)
   {
      EEPROM_Write(addr, 0xFF);
   }
}
// Function to read a byte from the EEPROM from ChatGPT
uint8_t EEPROM_read(uint16_t address) {
    // Wait for completion of previous write
    while (EECR & (1 << EEPE));

    // Set up address register
    EEAR = address;

    // Start EEPROM read by writing EERE
    EECR |= (1 << EERE);

    // Return data from the data register
    return EEDR;
}

// MARK: MAIN
int main (void) 
{
   MCUSR = 0;
   wdt_disable();
	slaveinit();
   
   int0_init();
	_delay_ms(10);
   timer2(4);
   //_delay_ms(10);
	// initialize the LCD 
	lcd_initialize(LCD_FUNCTION_8x2, LCD_CMD_ENTRY_INC, LCD_CMD_ON);
   _delay_ms(100);
	lcd_puts("Guten Tag\0");
	_delay_ms(1000);
	//lcd_cls();
   //_delay_ms(100);
	lcd_puts("H0-Decoder A328_PIO");
	
   
	
	//initADC(TASTATURPIN);
	
   uint16_t loopcount0=0;
   uint16_t loopcount1=0;

   uint16_t firstruncount0=0;
   //uint16_t firstruncount1=0;

   oldfunktion = 0x03; // 0x02
   oldlokdata = 0xCE;
   
   lcd_gotoxy(0,1);
   lcd_puts("ADR ");
   lcd_puthex(LOK_ADRESSE);
   lcd_putc(' ');
   lcd_hextobin(LOK_ADRESSE);
   
   _delay_ms(1000);
   lcd_cls();
   
   
   uint8_t counter = 0;
   uint16_t lcdcounter = 0;
    
    /*
   if (SHOWDISPLAY)
   {
      //setlogscreen();
   }
   */
  
   uint8_t i = 0;
   for (i=0;i<15;i++)
   {
      speedlookup[i] = speedlookuptable[speedindex][i];
   }
   maxspeed = speedlookup[14];
   minspeed = speedlookup[1];
   

   lcd_gotoxy(0,3);

   // naechste Adresse fuer save Status
   for (uint16_t loc = 0;loc < MAX_EEPROM; loc++)
   {   
      uint8_t locdata = EEPROM_Read(loc);
      if(locdata == 0xFF)
      {
         lcd_putint(loc);
         lcd_putc(' ');
         lcd_puthex(locdata);
         if(loc)
         {
            lcd_putc(' ');
            lcd_puthex(EEPROM_Read(loc - 1));
         }
         

         saveEEPROM_Addresse = loc;
         break;

      }
   }
   
   lcd_gotoxy(0,0);

   if(saveEEPROM_Addresse)
   {
      lcd_puts("last ");
      lcd_putc('A');
      lcd_putc(':');
      lcd_putint(saveEEPROM_Addresse);
      lcd_putc(' ');
      lcd_putc('S');
      lcd_putc(':');
      EEPROM_lastsavedstatus = EEPROM_Read(saveEEPROM_Addresse - 1);
      lcd_puthex(EEPROM_lastsavedstatus); // letzter gespeicherter Stetus
    // last data
   
      lcd_gotoxy(0,1);
      lcd_putc('L');
      lcd_putc(':');
      uint8_t lastlampecode = (EEPROM_lastsavedstatus & 0x03) ;
      lcd_puthex(lastlampecode);
      lcd_putc(' ');
      lcd_putc('D');
      lcd_putc(':');
      uint8_t lastdircode = (EEPROM_lastsavedstatus & 0x0C) >> 2;
      //lcd_putc('*');
      lcd_puthex(lastdircode);

      if (lastdircode == 1)
      {
         pwmpin = MOTORA_PIN;
         richtungpin = MOTORB_PIN;
         ledonpin = LAMPEA_PIN;
         ledoffpin = LAMPEB_PIN;
      }
      else if (lastdircode == 2)
      {
         pwmpin = MOTORB_PIN;
         richtungpin = MOTORA_PIN;
         ledonpin = LAMPEB_PIN;
         ledoffpin = LAMPEA_PIN;
      }
      /*
      lampe A: bit 1
      lampe B: bit 0

      motor A: bit 2 lastdircode = 1
      motor B: bit 3 lastdircode = 2
      */
      // speed
      /*
      uint8_t lastspeedcode = (EEPROM_lastsavedstatus & 0xF0) >> 4;
      lcd_putc(' ');
      lcd_putc('c');
      lcd_putc(':');
      lcd_putint2(lastspeedcode);
   
      lcd_putc(' ');
      lcd_putc('v');
      lcd_putc(':');
      lcd_putint(speedlookup[lastspeedcode]);
      */
   }
   else
   {
      //lcd_putc('*');
      //lcd_puts("first");
      pwmpin = MOTORA_PIN;
      richtungpin = MOTORB_PIN;
      ledonpin = LAMPEA_PIN;
      ledoffpin = LAMPEB_PIN;
   }

  
   sei();
   
	while (1)
   {  
      //OSZI_B_TOGG();
      // Timing: loop: 40 us, takt 85us, mit if-teil 160 us
      wdt_reset();
        // firstrun
      
      
         
         
               loopcount1++;

               // ************************************************
               // speedchangetakt
               // ************************************************
            if (loopcount1 >= speedchangetakt) // 150
            {
               //OSZI_B_LO();
                lcdcounter++;
               //LOOPLEDPORT ^= (1<<LOOPLED); // Kontrolle lastDIR
               loopcount1 = 0;
               
               
               // MARK: SPEED VAR
               // speed var
               
               if((newspeed > speed)) // beschleunigen, speedintervall positiv
               {
                  //OSZI_B_LO();
                  if(speed < (newspeed - speedintervall))
                  {
                     if((startspeed > speed) && (lokstatus & (1<<STARTBIT))) // Startimpuls
                     {
                        speed = startspeed;
                        lokstatus &= ~(1<<STARTBIT);
                     }
                     else 
                     {
                        speed += speedintervall;
                     }
                  }
                  else 
                  {
                     speed = newspeed;
                  }
                  //OSZI_B_HI();
               }
               else if((newspeed < speed)) // bremsen, speedintervall negativ
               {
                  uint8_t bremsintervall = (speedintervall * 2);
                  if((speed > newspeed ) && (bremsintervall > 0))
                  {
                     speed += bremsintervall; // : war 2*speedintervall
                     if(speed <= minspeed)
                     {
                        if((newspeed == 0) ) // Motor soll abstellen
                        {
                           speed = 0; // Motor OFF
                        } // if newspeed == 0

                     } //  if(speed <= minspeed/4)
                  
                  }
                  else 
                  {
                     speed = newspeed;
                  }
                  //OSZI_A_HI();
               } // newspeed < speed
               //OSZI_B_HI();
               // speed == 0 start
               
               if (speed == 0) // Stillstand erreicht
               {
                  if(richtungstatus & (1<<RICHTUNGCHANGESTARTBIT))
                  {
                     if(lokstatus & (1<<LOK_CHANGEBIT)) // Motor-Pins tauschen
                     {
                        //OSZI_B_LO();
                        EEPROM_savestatus &= ~0xF0;
                        EEPROM_savestatus |= ((speedcode & 0x0F) << 4);
                        if(pwmpin == MOTORA_PIN) // > auf MOTORB wechseln
                        {
                           pwmpin = MOTORB_PIN;
                           richtungpin = MOTORA_PIN;
                           EEPROM_savestatus &= ~(1<<MOTORA_PIN);
                           EEPROM_savestatus |= (1<<MOTORB_PIN);
                           
                           if(lokstatus & (1<<FUNKTIONBIT)) // Funktion ist 1, einschalten
                           {
                              EEPROM_savestatus |= (1<<LAMPEA_PIN);
                              EEPROM_savestatus &= ~(1<<LAMPEB_PIN);
                           }
                           else // funktion ist 0, ausschalten
                           {
                              // beide lampen OFF
                              EEPROM_savestatus &= ~(1<<LAMPEB_PIN);
                              EEPROM_savestatus &= ~(1<<LAMPEA_PIN);
                           }
                        }
                        else  if(pwmpin == MOTORB_PIN)// auch default
                        {
                           pwmpin = MOTORA_PIN;
                           richtungpin = MOTORB_PIN;
                           EEPROM_savestatus &= ~(1<<MOTORB_PIN);
                           EEPROM_savestatus |= (1<<MOTORA_PIN);

                           if(lokstatus & (1<<FUNKTIONBIT)) // Funktion ist 1, einschalten
                           {
                              EEPROM_savestatus |= (1<<LAMPEB_PIN);
                              EEPROM_savestatus &= ~(1<<LAMPEA_PIN);
                           }
                           else // funktion ist 0, ausschalten
                           {
                              // beide lampen OFF
                              EEPROM_savestatus &= ~(1<<LAMPEB_PIN);
                              EEPROM_savestatus &= ~(1<<LAMPEA_PIN);
                           }
                           
                        }
                        OSZI_B_HI();
                        
                        EEPROM_Write(saveEEPROM_Addresse,EEPROM_savestatus);
                        //OSZI_B_LO();
                        
                        lcd_gotoxy(0,2);
                        lcd_putint(saveEEPROM_Addresse);
                        lcd_putc(' ');
                        lcd_puthex(EEPROM_savestatus);
                        
                        if(saveEEPROM_Addresse > 20)
                        {
                           lcd_gotoxy(19,1);
                           lcd_putc('N');
                           EEPROM_Clear();
                           saveEEPROM_Addresse = 0;
                        }
                        else 
                        
                        {
                        //  lcd_gotoxy(19,1);
                        //  lcd_putc(' ');
                           saveEEPROM_Addresse++;
                        }
                        

                        MOTORPORT |= (1<<richtungpin); // Richtung setzen
                        
                        lokstatus &= ~(1<<LOK_CHANGEBIT);
                        lokstatus &= ~(1<<RUNBIT);

                        richtungstatus |= (1<<RICHTUNGCHANGEOKBIT); // Richtungswechsel erfolgt

                        OSZI_B_HI();
                        
                     } // if changebit
                     
                        lokstatus &= ~(1<<RICHTUNGBIT);

                     //OSZI_A_HI();

                  } // if(richtungstatus & (1<<RICHTUNGCHANGESTARTBIT))



               } // if speeed == 0
               
               // speed == 0 end





                  // 250103 lampen
               
               // Lampen einstellen
               if(ledstatus & (1<<LED_CHANGEBIT))
               {
                  if(richtungpin == MOTORA_PIN)
                  {
                     ledonpin = LAMPEB_PIN;
                     ledoffpin = LAMPEA_PIN;
                  }
                  else 
                  {
                     ledonpin = LAMPEA_PIN;
                     ledoffpin = LAMPEB_PIN;
                  }
                  //OSZI_B_LO();
                  if(lokstatus & (1<<FUNKTIONBIT))
                  {
                     
                     LAMPEPORT |= (1<<ledonpin); // Lampe         +
                     LAMPEPORT &= ~(1<<ledoffpin); // // Lampe    0
                     
                  }
                  else
                  {
                     // beide lampen OFF
                     //OSZI_A_HI();
                     LAMPEPORT &= ~(1<<ledonpin); // // eingeschaltete Lampe  OFF
                     
                  }
                  //OSZI_B_HI();
                  ledstatus &= ~(1<<LED_CHANGEBIT);
               }

               //displaydata[SPEED] = speed;
          
            } // loopcount1 >= speedchangetakt
            
         
         
         
         loopcount0++;
         if (loopcount0>= MAXLOOP0)
         {
            //OSZI_B_LO();
            
            LOOPLEDPORT ^= (1<<LOOPLED); 
            
            loopcount0=0;
 
            // Takt for display
             // MARK: LCD loop Display
            displaycounter1++;
            displaycounter1 = 0;
            if (displaycounter1 > 0x02)
            {
               displaycounter1=0;
               //LOOPLEDPORT ^= (1<<LOOPLED);
               if(TESTPIN & (1<<TEST1))
               {
                  lcd_gotoxy(16,1);
                  lcd_putint(counter);
                  lcd_gotoxy(0,2);
                  lcd_putint2(speedcode);
                  lcd_putc(' ');
                  lcd_putint(speedlookup[speedcode]);
                  lcd_gotoxy(17,2);
                  lcd_putint(speed);
               }
               else 
               {
                  lcd_gotoxy(17,0);
                  lcd_putint(speed);
                  lcd_gotoxy(17,2);
                  lcd_putint2(speedcode);
               }
  
               counter++;
               
               //               int0_init();
               //               sei();
               //EIMSK |= (1 << INT0); 
               /*
                char_x=80;
                char_y = 4;
                display_write_sec_min(lcdcounter, 1);
                */      
               //                lcd_gotoxy(16,1);
               //                lcd_putint(lcdcounter);
            }
             
            //OSZI_B_HI();
         }  // loopcount0>=refreshtakt
         //OSZI_B_HI();
         
      
   }//while


 return 0;
}
