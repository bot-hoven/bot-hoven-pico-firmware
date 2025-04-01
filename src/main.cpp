#include <stdio.h>
#include "pico/stdlib.h"
#include "include/MotorController.h"

int main() {
    
    // Create and initialize the motor controller
    MotorController controller;
    controller.init();
    
    // Main program loop
    while (true) {
        // Process updates (includes SPI command handling and motor position updates)
        controller.update();
        
        // Small delay to prevent tight looping
        // sleep_us(5000);
    }
    
    return 0;
}