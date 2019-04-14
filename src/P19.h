/*
    Définition des pins pour la carte P19
    Fichier P19.h V1.0

    Rolf Ziegler
    Mars 2019

*/
#ifndef __P19_h__
#define __P19_h__

#include "Arduino.h"

// interface moteur
    #define STPA 27
    #define STPB 33
    #define DIRA 14
    #define DIRB 25
    #define ENA  12    // Enable moteur pas à pas

// interface Neo-Pixel
    #define NEO  32    // Pin pour neo-pixels (1)

// ADC	mesure la tension d'entrée /33/133
	#define ADC_PIN 36 // ADC1_0, channel 1 pin 0


// I2C
    #define STA 21  // I2C
    #define SCL 22  // I2C

// SPI
    #define SCK 18
    #define MISO 19
    #define MOSI 23
    #define SS   5


// BUZZER
    #define BUZZER 26
    #define DAC    26
// interface TOUCH
	#define TC1 4
	#define TC2 0
	#define TC3 2
	#define TC4 15	// aussi Servo PWM2
	#define TC5 13	// aussi Servo PWM1

// servos
	#define SERVO1 13	// aussi Touch 4
	#define SERVO2 15	// aussi Touch 5
	#define SERVO3 27 // STPA
	#define SERVO4 14 // DIRA
	#define SERVO5 33 // STPB
	#define SERVO6 25 // DIRB

// diverses interruptions sur ESP32
    #define INTG    // interruption pour module Gyro
    #define INTM 34 // interruption pour circuit MCP23017
    #define INF  35 // interruption pour module LASER


// interruptions sur MCP23017
	#define INTA 0	// interruption circuit ADC
	#define INT_AU 1 // interruption circuit Audio externe sur connecteur UART2
	#define INTR 2   // interruption RTC, peut-être mis sur ESP avec SJ1 si pas de Laser
	#define INTB 4  // interruption du LASER Arrière

// autres pins sur MCP23017
	#define IREN 3 // enclencher les diodes sur capteurs IR (coins du circuit)
	#define DETSD 5 // detection si carte SD présente (niveau bas si présent)
						// rem, les détecteurs laser partagent la même adresse i2c
						// si 2 capteurs présent, un des 2 doit être désactivé
	#define SHUTB 6 // arrête laser arrière (Si 2 laser présents)
	#define SHUTF 7 // arrête laser avant (Si 2 laser présents)

// MCP23017, les pins du port B ne sont pas nommés, libre à chacun des les gérer


#endif