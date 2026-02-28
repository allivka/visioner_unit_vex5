#include "config.hpp"
#include "visioner_unit_vex5.hpp"
#include <stdio.h>
#include <Wire.h>

VisionerUnitVex5::Visioner visioner;
using VisionerUnitVex5::Buffer;
using VisionerUnitVex5::VisionerBehaviour;

int last;

void setup() {
    
    last = millis();
    
    pinMode(LED_BUILTIN, OUTPUT);
    
    Serial.begin(115200);
    Serial.setTimeout(1000);
    
    visioner.setup();
    
    visioner.setBehaviour(VisionerUnitVex5::VisionerBehaviour{}.set_speed(1000));
    
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    
}


void loop() {
    
    visioner.go();
    
    auto yaw = visioner.getYaw();
    
    if (yaw.isError()) return;
    
    double angle = yaw().deg();
    
    Serial.write((char*)&angle, sizeof(double));
    
    if(max(millis() - last, millis()) < 1000 / UART_HZ) return;
    
    Buffer buff;
    buff.data = vislib::core::UniquePtr<char>((char*)malloc(VisionerBehaviour::packetSize));
    buff.size = VisionerBehaviour::packetSize;
    
    Serial.readBytes(buff.data.get(), VisionerBehaviour::packetSize);
    
    VisionerBehaviour beh;
    beh.deserialize(buff);
    
    visioner.setBehaviour(beh);
    
    last = millis();
}