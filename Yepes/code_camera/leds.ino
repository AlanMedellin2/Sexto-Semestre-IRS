#include <Adafruit_NeoPixel.h>

#define PIN 2
#define NUMPIXELS 30   // pon aquí el número real de LEDs

Adafruit_NeoPixel tira(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  tira.begin();
  tira.setBrightness(255); // máximo brillo
  tira.clear();

  for (int i = 0; i < NUMPIXELS; i++) {
    tira.setPixelColor(i, tira.Color(255, 255, 255)); // blanco máximo
  }

  tira.show();
}

void loop() {
}
