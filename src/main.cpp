#include "visioner_unit_vex5.hpp"
#include <stdio.h>
#include <Wire.h>

VisionerUnitVex5::Visioner visioner;

void setup() {
    
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.begin(115200);
    
    visioner.setup();
    
    visioner.setBehaviour(VisionerUnitVex5::VisionerBehaviour{}.set_speed(1000));
    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    
}

void loop() {
    visioner.go();
}