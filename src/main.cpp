#include <stdio.h>
#include "pico/stdlib.h"
#include "include/MotorController.h"
#include "pico/multicore.h"

int main() {
    // Initialize controller
    MotorController controller;
    controller.init();
    printf("Launching core...\n");
    
    // Split duties between cores
    multicore_launch_core1(MotorController::core1_entry_static);

    printf("Made past launch core1\n");
    
    // Core 0 handles SPI and left stepper
    while (true) {
        controller.update_core0();
        sleep_us(50);
    }
    
    return 0;
}




// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "include/MotorController.h"
// #include "pico/multicore.h"

// // Debug mode flag - set to 1 to enable debug prints, 0 to disable
// #define DEBUG_MODE 0

// /**
//  * @brief Main entry point for the application
//  * 
//  * Initializes the motor controller, launches core1, and runs the
//  * core0 update loop indefinitely.
//  * 
//  * @return int Return code (never reached)
//  */
// int main() {
//     // Initialize controller
//     MotorController controller;
//     controller.init();
    
//     if (DEBUG_MODE) {
//         printf("Launching core...\n");
//     }
    
//     // Split duties between cores
//     multicore_launch_core1(MotorController::core1_entry_static);

//     if (DEBUG_MODE) {
//         printf("Made past launch core1\n");
//     }
    
//     // Core 0 handles SPI and left stepper
//     while (true) {
//         controller.update_core0();
//         sleep_us(50);
//     }
    
//     return 0;
// }