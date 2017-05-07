#include <digitalWriteFast.h> // DigitalWriteFast Library

#define RED_PIN 3
#define GREEN_PIN 5
#define BLUE_PIN 9

#define RC_PIN 2

enum colors {
  RED,
  GREEN,
  BLUE,
  RAINBOW
};
colors eyesMode = RAINBOW;

void setup() {
  pinMode(RC_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(RC_PIN), calcRC, CHANGE); 
}

void loop() {
  static int rgb[3];
  static int dec = 0;
  static int counter = 0;

  switch(eyesMode) {
    case RED:
        writeRgb(255, 0, 0);
        break;
    case GREEN:
        writeRgb(0, 255, 0);
        break;        
     case BLUE:
        writeRgb(0, 0, 255);
        break; 
     case RAINBOW:
        if ((dec==0)&&(counter==0)) {
          rgb[0] = 255;
          rgb[1] = 0;
          rgb[2] = 0;         
        }
        writeRgb(rgb[0], rgb[1], rgb[2]);
        delay(25);
        
        int inc = (dec + 1) % 3;
        rgb[dec]--;
        rgb[inc]++;

        counter++;
        if (counter==255) {
          counter = 0;
          dec = (dec + 1) % 3;
        }
  }
}

void writeRgb(int red, int green, int blue) {
  analogWrite(RED_PIN, 255-red);  // Invert for common anode
  analogWrite(GREEN_PIN, 255-green);
  analogWrite(BLUE_PIN, 255-blue);
}

void calcRC()
{
  static unsigned int RC_Start;
  
  // if the pin is high, its a rising edge of the signal pulse,
  // so lets record its value
  if (digitalReadFast(RC_PIN) == HIGH)
  {
    RC_Start = micros();
  }
  else
  {
    // else it must be a falling edge, so lets get the time and subtract the time
    // of the rising edge this gives use the time between the rising and falling
    // edges i.e. the pulse duration.
    uint16_t RC_In = (uint16_t)(micros() - RC_Start);
    
    if (RC_In < 1250) {
      eyesMode = BLUE;
    } else if (RC_In < 1500) {
      eyesMode = RED;
    } else if (RC_In < 1750) {
      eyesMode = GREEN;
    } else {
      eyesMode = RAINBOW;
    }
  }
}



