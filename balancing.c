// Selbstbalancierender Roboter
// Gerhard Steininger
// Bachelorarbeit - 05. März 2015

// Hinweise: Regelparameter müssen individuell angepasst werden

#include <stm32f10x.h>                       /* STM32F100 Definitionen */
#include <stdio.h>
#include <math.h>
#include "I2C.h"
#include "kalman.h"

#define I2C_Clockspeed 400000								// Taktrate I2C-Bus
#define I2C_OwnAddress 0x50									// eigene I2C-Bus-Adresse - Master
#define I2C_MPUAddress 0xD1									// MPU-6050 I2C-Bus-Adresse - Slave   
#define I2C1 ((I2C_TypeDef *) I2C1_BASE)		// Zeiger auf I2C1-Registersatz

#define sys_8 0.008													// 8ms
#define sys_ms8 3000*8											// 8ms für SysTick timer Konfiguration

#define pi 3.14159
#define RAD_TO_DEG 57.2957786								// Umrechnung von Rad zu Grad

#define GYRO_SCALE_FACTOR 131.0							// Skalierungsfaktor der Drehratensensors

uint8_t i2cData[14];												// Register des MPU-6050 beschreiben
uint8_t i2cDataRead[14];										// Werte vom MPU-6050 speichern
uint8_t PLL [2];

int i;																			// Zählvariable
int16_t accY, accZ, gyroX;									// Beschleunigungswerte Y und Z, Winkelgeschwindigkeit X
double gyroXrate;														// Winkelgeschwindigkeit nach Saklierung

double accYangle = 0.0;											// Winkel aus Beschleunigungswerten
double angleOffset = 0.6;										// Winkel Offset
double gyroAngle = 0.0;											// Winkel aus Winkelgeschwindigkeit

double compAngle = 0.0;											// Komplementärwinkel
double kalAngle = 0.0;											// Kalman-Winkel

double LOutput = 0.0;												// Polung des linken Motors
double ROutput = 0.0;												// Polung des rechten Motors

// Regelparameter
double kp = 22; 														// Regelparameter kp							
double ki = 0.5;														// Regelparameter ki							
double kd = 0.05;														// Regelparameter kd							
double eSum=0, dE=0, e=0, last_e=0; 				// Hilfsvariablen für die Regelung
double Output;															// Regler-Ausgangssignal

double LMotor_offset = 20;									// Motoroffset links
double RMotor_offset = 20;									// Motoroffset rechts
int ABS_Loutput = 0;												// PWM-Ausgangssignal links
int ABS_Routput = 0;												// PWN-Ausgangssignal rechts

int LOutpFlag = 0;													// Hilfsvariable um Polarität des linken Motors zu ändern
int ROutpFlag = 0;													// Hilfsvariable um Polarität des rechten Motors zu ändern

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
// abs Funktion - gibt den Betrag vom übergebenen Wert zurück
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
double abs(double val) {
	if(val < 0)
		return val*(-1);
	
	else
		return val;
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
// SysTick timer initialisieren - alle 8 ms auslösen
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
void InitSysTick(void) {
	  //vom SysTick timer wird regelmäßig nach Ablauf des Zählers ein Interrupt ausgelöst -> dazu auch Vektortabelle beachten und ISR s. unten
	
		SysTick->LOAD = sys_ms8; 								// 3000*8 = 24000
		SysTick->CTRL = 0x02;										// Bit 2 = 0 -> AHB/8 (Processor_clock / 8) -> 24MHz/8 = 3MHz
																						// Bit 1 = 1 -> sobald vollständig runtergezählt wird ein Interrupt ausgelöst
																						// Bit 0 = 0 -> Zähler noch abgeschalten
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Timer2 initialisieren - Ausgabe des PWM Signals
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
void InitTIM2_PWM(void)
{
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;   	// GPIOA Takt einschalten (Bit IOPAEN)
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;   	// Timer2 Takt einschalten (Bit TIM2EN)
    GPIOA->CRL = 0x4444AAAA;              	// PA0...3 auf Alternate Function Output, Push-Pull

		TIM2->EGR = 0x01;												// Timer-Register initialisieren (Bit UG)
		TIM2->CCMR1 = 0x7878;   								// Output Compare Mode: PWM Mode2(OC1M=111), Output Compare preload enable (OC1PE=1)
																						// 0x7878 = 0b 0111 1000 0111 1000 -> OC1 und OC2 konfigurieren
    TIM2->CCER = 0x3333;										// Capture/Compare output polarity -> active low;
		
		TIM2->PSC = 187;												// 255 sollen 2ms entsprechen
																		
		TIM2->ARR = 255;												// ca. 2ms Periode
		TIM2->CCR1 = 0;													// Kanal1 Vergleichswert
		TIM2->CCR2 = 0;													// Kanal2 Vergleichswert
		
		TIM2->CR1 = 0x01;												// Zähler aktivieren
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
// MPU-6050 initialisieren
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
void Init_MPU6050(void) {
	
		// Abtastrate des Sensors einstellen - Register 0x19 mit 7 beschreiben
		i2cData[0] = 0x19;
		i2cData[1] = 7;
		I2C_Write(I2C1, i2cData, 2, I2C_MPUAddress);
			
		// internen Tiefpassfilter deaktivieren - Register 0x1A mit 0 beschreiben
		i2cData[0] = 0x1A;
		i2cData[1] = 0x00;
		I2C_Write(I2C1, i2cData, 2, I2C_MPUAddress);
			
		// max. Messbereich des Drehratensensors auf +/-250°/s einstellen - Register 0x1B mit 0 beschreiben
		i2cData[0] = 0x1B;
		i2cData[1] = 0x00;
		I2C_Write(I2C1, i2cData, 2, I2C_MPUAddress);
			
		// max. Messbereich des Beschleunigungssensors auf +/-2g einstellen - Register 0x1C mit 0 beschreiben
		i2cData[0] = 0x1C;
		i2cData[1] = 0x00;
		I2C_Write(I2C1, i2cData, 2, I2C_MPUAddress);
			
		// Taktquelle auswählen - Register 0x6B mit 1 beschreiben
		PLL[0] = 0x6B;
		PLL[1] = 0x01;
		I2C_Write(I2C1, PLL, 2, I2C_MPUAddress);
}

//------------------------------------------------------------------------------------------------------------------------------------------------------------------
// PWMControl - Ausgabe und Richtung
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
void PWMControl() {
	
		//LOutput = linker Motor -> Polarität festlegen
		if(LOutput < 0) {
			if(LOutpFlag == 2 || LOutpFlag == 0) {
				GPIOC->ODR &= 0xFC;											// betreffende Bits löschen um diese später neu zu beschreiben
				LOutpFlag = 1;
			}
			GPIOC->ODR |= (1UL << 1)|(0 << 0);				// IN1 low, und IN2 high
		}
		else if(LOutput > 0) {
			if(LOutpFlag == 1 || LOutpFlag == 0) {
				GPIOC->ODR &= 0xFC;											// betreffende Bits löschen um diese später neu zu beschreiben
				LOutpFlag = 2;
			}
			GPIOC->ODR |= (0 << 1)|(1UL << 0);				// IN1 high, und IN2 low
		}
		else {
			GPIOC->ODR |= (1UL << 0)|(1UL << 1);			// IN1 high und IN2 high
			LOutpFlag = 0;
		}
		
		//ROutput = rechter Moter -> Polarität festlegen
		if(ROutput < 0) {
			if(ROutpFlag == 2 || ROutpFlag == 0) {
				GPIOC->ODR &= 0xF3;											// betreffende Bits löschen um diese später neu zu beschreiben
				ROutpFlag = 1;
			}
			GPIOC->ODR |= (1UL << 2)|(0 << 3);				// IN3 high und IN4 low
		}	
		else if(ROutput > 0) {
			if(ROutpFlag == 1 || ROutpFlag == 0) {
				GPIOC->ODR &= 0xF3;											// betreffende Bits löschen um diese später neu zu beschreiben
				ROutpFlag = 2;
			}
			GPIOC->ODR |= (0 << 2)|(1UL << 3);				// IN3 low und IN4 high
		}
		else {
			GPIOC->ODR |= (1UL << 2)|(1UL << 3);			// IN3 high und IN4 high
			ROutpFlag = 0;
		}
		
		// PWM-Signal für linken Motor begrenzen wenn Wert größer 255
		ABS_Loutput = abs(LOutput) + LMotor_offset;
		if(ABS_Loutput > 255)
			ABS_Loutput = 255;
		
		// PWM-Signal für rechten Motor begrenzen wenn Wert größer 255
		ABS_Routput = abs(ROutput) + RMotor_offset;
		if(ABS_Routput > 255)
			ABS_Routput = 255;
		
		TIM2->CCR1 = ABS_Loutput;										// PWM-Signal für linken Motor ausgeben
		TIM2->CCR2 = ABS_Routput;										// PWM-Signal für rechten Motor ausgeben
}


//----------------------------------------------------------------------------------------------------------------------------------------
// Hauptprogramm
//----------------------------------------------------------------------------------------------------------------------------------------
int main (void)
{ 	
	// I2C initialisieren
	I2C_LowLevel_Init(I2C1, I2C_Clockspeed , I2C_OwnAddress);		// I2C initialisieren -> Kanal 1, clockspeed, Materadresse
	
	// Systick timer initialisieren
  InitSysTick();																							// SysTick timer initialisieren (Ladewert, Clockspeed)
		
	// Timer2 initialisieren
	InitTIM2_PWM();																							// Timer 2 initialisieren -> notwendig um PWM Signal auszugeben
	
	// Aktiviere Port C für Motor Polarität
	RCC->APB2ENR |= (1UL << 4);     														// Enable GPIOC
	GPIOC->CRL		=  0x33333333;    														// PortC 0..7 als Output -> Output Mode, max speed 50MHz ; General purpose output Open-drain
	
	// MPU-6050 initialisieren
	Init_MPU6050();
	
	// Beschleunigungswerte und Drehraten vom MPU-6050 lesen
	i2cData[0] = 0x3B;
	I2C_Write(I2C1, i2cData, 1, I2C_MPUAddress);
	I2C_Read(I2C1, i2cDataRead, 14, I2C_MPUAddress);
	
	// Daten ordnen und abspeichern
	accY = ((i2cDataRead[2] << 8) | i2cDataRead[3]);
	accZ = ((i2cDataRead[4] << 8) | i2cDataRead[5]);
	gyroX = ((i2cDataRead[8] << 8) | i2cData[9]);
	
	//Kalibrierung von accY, accZ und gyroX - Werte wurden in einem Versuch ermittelt
	accY = accY - 150;
  accZ = accZ - 451.63;
  gyroX = gyroX + 273.25;
	
	//Winkel initialisieren
	accYangle = (atan2(accY, accZ))*RAD_TO_DEG; 				// Winkel aus Beschleunigungswerten berechnen
	setAngle(accYangle);																// Kalman-(Start)Winkel setzen
	gyroAngle = accYangle;															// Drehratenwinkel initialisieren
	compAngle = accYangle;															// Komplementärwinkel initialisieren
	
	SysTick->CTRL |= 0x01; 															// Systick timer einschalten - sonst würde dieser bereits vor der Inittialisierung auslösen
	
	
	//while-Schleife um auf den Interrupt des Systick timers zu warten
	while(1) {
		
	}//while

}//int main


//------------------------------------------------------------------------------------------------------------------------------------------------------------------
// Interrupt-Handler des Systick timers
//------------------------------------------------------------------------------------------------------------------------------------------------------------------
void SysTick_Handler(void)		
{
	GPIOC->ODR |= (1UL << 5);														// Pin5 an Port C oszilloskopieren um dauer für den Systick Handler zu beobachten (optional)
	
	// Beschleunigungswerte und Drehraten vom MPU-6050 lesen
	i2cData[0] = 0x3B;																	// Register 0x3B stösst das Auslesen an
	I2C_Write(I2C1, i2cData, 1, I2C_MPUAddress);
	I2C_Read(I2C1, i2cDataRead, 14, I2C_MPUAddress);
	
	// Daten ordnen und abspeichern
	accY = ((i2cDataRead[2] << 8) | i2cDataRead[3]);					// gelesene Daten in Variable speichern
	accZ = ((i2cDataRead[4] << 8) | i2cDataRead[5]);					// gelesene Daten in Variable speichern
	gyroX = ((i2cDataRead[8] << 8) | i2cData[9]);							// gelesene Daten in Variable speichern
		
	//Kalibrierung von accY, accZ und gyroX - Werte wurden in einem Versuch ermittelt
	accY = accY + 150;
	accZ = accZ - 451.63;
	gyroX = gyroX + 273.25;
		
	accYangle = (atan2(accY, accZ))*RAD_TO_DEG; 							// Winkel aus den Beschleunigungswerten errechnen
	accYangle = accYangle + angleOffset;											// Winkeloffset addieren
	gyroXrate = gyroX / GYRO_SCALE_FACTOR;										// Winkelgeschwindigkeit nach Skalierung
			
	// erneute Initialisierung der Winkel bei Vorzeichenwechsel
	if((accYangle < -90 && kalAngle > 90) || (accYangle > 90 && kalAngle < -90)) {
		setAngle(accYangle);
		compAngle = accYangle;
		kalAngle = accYangle;
		gyroAngle = accYangle;
	}
	// Kalman-Winkel mit Kalman-Funktion erhalten
	else
		kalAngle = getAngle(accYangle, gyroXrate, sys_8);
	
	gyroAngle = gyroAngle + gyroXrate * sys_8;								// Winkel aus den Winkelgeschwindigkeitswerten errechnen
																															
	compAngle = 0.93 * (compAngle + gyroXrate * sys_8) + 0.07 * accYangle;			// Winkel über Komplementärfilter berechnen
	
	// Drift des Drehratensensors abfangen
	if((gyroAngle < -180) || (gyroAngle > 180))
		gyroAngle = kalAngle;
	
	if(abs(kalAngle) < 45) {																	// erkennen ob Roboter umgefallen -> wenn < 45, dann nicht umgefallen
		
		//Relegung
		e = kalAngle;																						// Sollwinkelvorgabe
														
		eSum = eSum + e * sys_8;																// Integration -> I-Anteil
		dE = (e - last_e) / sys_8;															// Differentiation -> D-Anteil
	
		Output = kp * e + ki * eSum + kd * dE;									// Reglerausgang berechnen
	
		LOutput = Output;																				// Ausgangswert für linken Motor
		ROutput = Output;																				// Ausgangswert für rechten Motor
		
		last_e = e;																							// Aktualisierung des Eingangsparameters
				
		PWMControl();																						// PWM-Signal ausgeben
	} //if
	
	else {																										// erkennen ob Roboter umgefallen -> wenn >= 45, dann umgefallen
		TIM2->CCR1 = 0;																					// linken Motor abschalten
		TIM2->CCR2 = 0;																					// rechten Motor abschalten
		while(1) {}																							// Endlosschleife -> Reset notwendig
	} //else
	
		
	GPIOC->ODR &= 0xDF;																				// Pin5 an Port C oszilloskopieren um dauer für den Systick Handler zu beobachten (optional)
		
} //Interrupt-Handler des Systick timers
