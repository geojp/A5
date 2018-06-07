#include <Arduino.h>
#include <SPI.h>
#include "Adafruit_BLE.h"
#include "Adafruit_BluefruitLE_SPI.h"
#include "Adafruit_BluefruitLE_UART.h"
#include <Adafruit_CircuitPlayground.h>
#include "BluefruitConfig.h"

Adafruit_BluefruitLE_UART ble(BLUEFRUIT_HWSERIAL_NAME, BLUEFRUIT_UART_MODE_PIN);

void error(const __FlashStringHelper*err) {
  Serial.println(err);
  while (1);
}

void setup() {
  CircuitPlayground.begin();
  delay(500);

  Serial.begin(115200);
  Serial.println(F("Gesturer"));
  Serial.println(F("------------------------------------------------"));

  if ( !ble.begin(false) )
  {
    error(F("Couldn't find Bluefruit, make sure it's in CoMmanD mode & check wiring?"));
  }

  /* Disable command echo from Bluefruit */
  ble.echo(false);
  ble.verbose(false);  // debug info is a little annoying after this point!

  /* Wait for connection */
  while (! ble.isConnected()) {
      delay(500);
  }

  // Set module to DATA mode
  Serial.println( F("Connected!") );
  ble.setMode(BLUEFRUIT_MODE_DATA);

  Serial.println(F("******************************"));
}

char readBuffer[10];
int numRead = 0;
void waitForStart()
{
	while (1)
	{
		while(!ble.available()) 
		{
			delay(10);
		}
    
		// waiting for "Start"
		numRead += ble.readBytes(readBuffer + numRead, sizeof(readBuffer));
		readBuffer[5] = 0;

		if (strcmp(readBuffer, "Start") == 0)
		{
			return;
		}
	}
}

#define SAMPLES_SIZE  30
#define NUM_AXIS 			3
#define AXIS_X				0
#define AXIS_Y				1
#define AXIS_Z				2

double sampledA[NUM_AXIS][SAMPLES_SIZE];
void getSamples()
{
  for (int i = 0; i < SAMPLES_SIZE; i++)
	{
		sampledA[AXIS_X][i] = CircuitPlayground.motionX();
		sampledA[AXIS_Y][i] = CircuitPlayground.motionY();
		sampledA[AXIS_Z][i] = CircuitPlayground.motionZ();
		delay(50);
	}
}

#define NUM_FEAT					4

#define FEAT_AXIS_X				0
#define FEAT_AXIS_Y				(FEAT_AXIS_X + NUM_FEAT)
#define FEAT_AXIS_Z				(FEAT_AXIS_Y + NUM_FEAT)

#define FEAT_MEAN					0
#define FEAT_VAR					1
#define FEAT_ENERGY       2
#define FEAT_KURT         3

double featVector[NUM_FEAT*NUM_AXIS] = {0};
void calcFeatures()
{
	for (int i = 0; i < SAMPLES_SIZE; i++)
	{
		// calc sums
		featVector[FEAT_AXIS_X + FEAT_MEAN] += sampledA[AXIS_X][i];
		featVector[FEAT_AXIS_Y + FEAT_MEAN] += sampledA[AXIS_Y][i];
		featVector[FEAT_AXIS_Z + FEAT_MEAN] += sampledA[AXIS_Z][i];
		
    // calc energy mean
    featVector[FEAT_AXIS_X + FEAT_ENERGY] += abs(sampledA[AXIS_X][i]);
    featVector[FEAT_AXIS_Y + FEAT_ENERGY] += abs(sampledA[AXIS_Y][i]);
    featVector[FEAT_AXIS_Z + FEAT_ENERGY] += abs(sampledA[AXIS_Z][i]);
	}
	featVector[FEAT_AXIS_X + FEAT_MEAN] /= SAMPLES_SIZE;
	featVector[FEAT_AXIS_Y + FEAT_MEAN] /= SAMPLES_SIZE;
	featVector[FEAT_AXIS_Z + FEAT_MEAN] /= SAMPLES_SIZE;

  featVector[FEAT_AXIS_X + FEAT_ENERGY] /= SAMPLES_SIZE;
  featVector[FEAT_AXIS_Y + FEAT_ENERGY] /= SAMPLES_SIZE;
  featVector[FEAT_AXIS_Z + FEAT_ENERGY] /= SAMPLES_SIZE;
  
	for (int i = 0; i < SAMPLES_SIZE; i++)
	{
    // calc var
		featVector[FEAT_AXIS_X + FEAT_VAR] += pow(sampledA[AXIS_X][i] - featVector[FEAT_AXIS_X + FEAT_MEAN], 2);
		featVector[FEAT_AXIS_Y + FEAT_VAR] += pow(sampledA[AXIS_Y][i] - featVector[FEAT_AXIS_Y + FEAT_MEAN], 2);
		featVector[FEAT_AXIS_Z + FEAT_VAR] += pow(sampledA[AXIS_Z][i] - featVector[FEAT_AXIS_Z + FEAT_MEAN], 2);

    // calc kurtosis
    featVector[FEAT_AXIS_X + FEAT_KURT] += pow(sampledA[AXIS_X][i] - featVector[FEAT_AXIS_X + FEAT_MEAN], 4);
    featVector[FEAT_AXIS_Y + FEAT_KURT] += pow(sampledA[AXIS_Y][i] - featVector[FEAT_AXIS_Y + FEAT_MEAN], 4);
    featVector[FEAT_AXIS_Z + FEAT_KURT] += pow(sampledA[AXIS_Z][i] - featVector[FEAT_AXIS_Z + FEAT_MEAN], 4);
	}
	featVector[FEAT_AXIS_X + FEAT_VAR] /= SAMPLES_SIZE;
	featVector[FEAT_AXIS_Y + FEAT_VAR] /= SAMPLES_SIZE;
	featVector[FEAT_AXIS_Z + FEAT_VAR] /= SAMPLES_SIZE;

  featVector[FEAT_AXIS_X + FEAT_KURT] /= SAMPLES_SIZE * pow(sqrt(featVector[FEAT_AXIS_X + FEAT_VAR]), 4);
  featVector[FEAT_AXIS_Y + FEAT_KURT] /= SAMPLES_SIZE * pow(sqrt(featVector[FEAT_AXIS_Y + FEAT_VAR]), 4);
  featVector[FEAT_AXIS_Z + FEAT_KURT] /= SAMPLES_SIZE * pow(sqrt(featVector[FEAT_AXIS_Z + FEAT_VAR]), 4);

}

void loop() {
	waitForStart();
	
	CircuitPlayground.setPixelColor(0, 255,   0,   0);
  CircuitPlayground.setPixelColor(1, 255,   0,   0);
  CircuitPlayground.setPixelColor(2, 255,   0,   0);
  CircuitPlayground.setPixelColor(3, 255,   0,   0);
	getSamples();
  CircuitPlayground.clearPixels();
  
	calcFeatures();
	ble.print("FStart ");
	for (int i = 0; i < NUM_FEAT*NUM_AXIS; i++)
	{
		ble.print(featVector[i]);
		ble.print(" ");
	}
	ble.print("FStop");
}
