#pragma once

#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "PID.h"

#define ONE_WIRE_BUS 8
#define RELAY_PIN_1 A3
#define SSR_PIN 9
#define BUTTON_PIN 3

OneWire oneWire(ONE_WIRE_BUS);
DallasTemperature DS18B20(&oneWire);

LiquidCrystal_I2C lcd(0x27, 20, 4);

PIDv3 pid;

unsigned long prevMillis = 0;
unsigned long interval = 3000;

bool systemState = false;

float temperature = 0.0;
float temperatureTest = 0.0;
float sp = 40.0;
float outputPID;
