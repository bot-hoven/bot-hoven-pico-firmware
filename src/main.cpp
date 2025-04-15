#include <stdio.h>
#include "pico/stdlib.h"
#include "include/MotorController.h"
#include "pico/multicore.h"

// Entry point for program
int main() {
    // Initialize stdio for debug output
    stdio_init_all();
    // Wait for serial connection to stabilize
    sleep_ms(2000);
    
    printf("Starting Bot-Hoven Motor Controller\n");
    
    // Initialize LED for visual debugging
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
    
    // Turn on LED to indicate startup
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    
    // Create and initialize the motor controller
    printf("Creating motor controller...\n");
    MotorController controller;
    
    printf("Initializing motor controller...\n");
    controller.init();
    
    // Turn on LED again to indicate controller initialized
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
    sleep_ms(500);
    gpio_put(PICO_DEFAULT_LED_PIN, 0);
    
    printf("Launching Core 1...\n");
    // Launch Core 1 to handle right motor
    multicore_launch_core1(MotorController::core1_entry_static);
    
    printf("Core 1 launched. Entering main loop on Core 0.\n");
    
    // Main program loop on Core 0
    while (true) {
        // Process core 0 tasks (left motor and SPI)
        controller.update_core0();
        
        // Small delay to prevent tight looping
        sleep_us(100);
    }
    
    return 0;
}










// // #include <stdio.h>
// // #include "pico/stdlib.h"
// // #include "pico/multicore.h"
// // #include "include/MotorController.h"

// // // Global shared motor controller instance
// // MotorController* g_controller = nullptr;

// // // Flag for signaling calibration events
// // volatile bool g_calibrate_right_motor = false;
// // volatile bool g_right_motor_calibration_complete = false;

// // // Function that will run on Core 1
// // void core1_right_motor_task() {
// //     // Initialize core 1
// //     printf("Core 1 started - handling right motor\n");
    
// //     // Infinite loop to continuously process the right motor
// //     while (true) {
// //         // Check if controller has been initialized
// //         if (g_controller) {
// //             // Check if calibration is requested
// //             if (g_calibrate_right_motor) {
// //                 g_controller->calibrateRightMotor();
// //                 g_right_motor_calibration_complete = true;
// //                 g_calibrate_right_motor = false;
// //             }
            
// //             // Update only the right motor
// //             g_controller->updateRightMotor();
// //         }
        
// //         // Small delay to prevent tight looping
// //         sleep_us(50);
// //     }
// // }

// // int main() {
// //     // Initialize stdio for debugging output
// //     stdio_init_all();
// //     sleep_ms(2000); // Wait for serial connection to stabilize
    
// //     // Create and initialize the motor controller
// //     MotorController controller;
// //     g_controller = &controller;
// //     controller.init();
    
// //     printf("Launch the core\n");
// //     // Launch Core 1 to handle right motor exclusively
// //     multicore_launch_core1(core1_right_motor_task);
    
// //     // Main program loop on Core 0
// //     while (true) {
// //         // Process updates for left motor and SPI
// //         controller.updateLeftMotorAndSPI();
        
// //         // Small delay to prevent tight looping
// //         sleep_us(50);
// //     }
    
// //     return 0;
// // }


// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "include/MotorController.h"
// #include "pico/multicore.h"

// volatile bool g_calibrate_right_motor;
// volatile bool g_right_motor_calibration_complete;

// int main() {
//     // Initialize controller
//     MotorController controller;
//     controller.init();
//     printf("Launching core...\n");
    
//     // Split duties between cores
//     multicore_launch_core1(MotorController::core1_entry_static);

//     printf("Made past launch core1\n");
    
//     // Core 0 handles SPI and left stepper
//     while (true) {
//         controller.update_core0();
//         sleep_us(50);
//     }
    
//     return 0;
// }