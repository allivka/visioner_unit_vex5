#include "visioner_unit_vex5.hpp"
#include <stdio.h>
#include <Wire.h>

VisionerUnitVex5::Visioner visioner;

struct SerialBuffer {
    char buffer[VisionerUnitVex5::VisionerBehaviour::packetSize] = {};
    int8_t counter = 0;
    
    bool process() {
        int byte = Serial.read();
        
        if (byte == -1) return false;
        
        buffer[counter++] = (char)byte;
        
        if (counter == VisionerUnitVex5::VisionerBehaviour::packetSize) return true;
        
        return false;
    }
    
    void reset() {
        counter = 0;
    }
    
    VisionerUnitVex5::VisionerBehaviour get() {
        VisionerUnitVex5::Buffer buff;
        buff.size = counter;
        buff.data = vislib::core::UniquePtr<char>(buffer);
        
        return VisionerUnitVex5::VisionerBehaviour().deserialize(buff);
    }
};

SerialBuffer serialBus;

void setup() {
    
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
    
    if(serialBus.process()) {
        visioner.setBehaviour(serialBus.get());
        serialBus.reset();
    }
    
    auto yaw = visioner.getYaw();
    
    if (yaw.isError()) return;
    
    double angle = yaw().deg();
    
    Serial.write((char*)&angle, 8);
}