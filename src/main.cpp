// SPI to use (we use the default one at index 0)
#define LCD_HOST 0
//
// WIRING
//
// LCD screen
#define LCD_CS 10
#define LCD_DC 8
#define LCD_RST 9
#define LCD_BKL 7
#define LCD_ROTATION 1
// wire the remaining LCD SPI
// pins to 
// MOSI (11)
// MISO (12) (where available)
// SCK (13)

#include <Arduino.h>
#include <Wire.h>
#include <SD.h>
#include <teensy_gfx.hpp>
#include <gfx_cpp14.hpp>
#include "PaulMaul.hpp"
using namespace gfx;

teensy_gfx lcd(0, 10,8,9);
//ILI9341_t3n lcd(10,8,9);
using color_t = color<decltype(lcd)::pixel_type>;
void setup() {
    lcd.initialize();
    pinMode(7,OUTPUT);
    digitalWrite(7,HIGH);
    Serial.begin(115200);
    draw::filled_rectangle(lcd,lcd.bounds(),color_t::purple);
    open_text_info t;
    t.font = &PaulMaul;
    t.scale = PaulMaul.scale(50);
    t.text = "Hello";
    t.transparent_background=false;
    draw::text(lcd,lcd.bounds(),t,color_t::black,color_t::purple);
}
void loop() {

}