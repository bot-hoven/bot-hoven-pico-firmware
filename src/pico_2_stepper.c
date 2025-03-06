#include "include/pi_pico_stepper.h"

// Motor instances
StepperMotor l_stepper = {0.0, 0.0, 0.1};
StepperMotor r_stepper = {0.0, 0.0, 0.1};

// SPI buffers
uint8_t recv_buffer[BUFFER_SIZE] = {0};
char response_buffer[32] = {0};

// Perform initialisation
int pico_led_init(void) {
    gpio_init(LED_PIN);
    gpio_set_dir(LED_PIN, GPIO_OUT);
}

// Turn the led on or off
void pico_set_led(bool led_on) { gpio_put(LED_PIN, led_on); }

void spi_slave_init(spi_inst_t* spi_instance) {
    // You must ensure that the clock speed, data bits, and mode are the same on both master and slave
    spi_init(spi_instance, SPI_CLOCK_SPEED_HZ);
    spi_set_format(spi_instance, SPI_DATA_BITS, SPI_MODE_POLARITY, SPI_MODE_PHASE, SPI_BIT_ORDER);
    spi_set_slave(spi_instance, true);

    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_CS, GPIO_FUNC_SPI);
}

void update_position(StepperMotor* motor) {
    // Determine the amount to move the motor
    double position_change = motor->position_command - motor->position_state;

    // Move the motor if the change is greater than the resolution
    if (fabs(position_change) >= motor->resolution) {
        // Move the motor in the correct direction based on the sign of the change
        motor->position_state += (position_change > 0) ? motor->resolution : -motor->resolution;
    }
}

void handle_position_command(uint8_t motor) {
    uint8_t byte = 0xFF;
    uint8_t recv_index = 0;

    // Read the position command from the master until we reach the null-terminator or the buffer limit
    while (byte != 0x00 && recv_index < BUFFER_SIZE - 1) {
        spi_read_blocking(SPI_PORT, DUMMY, &byte, 1);
        recv_buffer[recv_index++] = byte;
    }
    recv_buffer[recv_index] = '\0';

    // Parse the position command from the buffer
    double position;
    // convert the command to a double using sscanf (success returns a 1)
    if (sscanf((char*)recv_buffer, "%lf", &position) == 1) {
        // Get the motor instance based on the motor identifier
        StepperMotor* target = (motor == 'l') ? &l_stepper : &r_stepper;
        target->position_command = position;
        printf("Set %c stepper to %.3f\n", motor, position);
    }
}

void handle_position_state_request(uint8_t motor) {
    // Get the motor instance based on the motor identifier
    StepperMotor* target = (motor == 'l') ? &l_stepper : &r_stepper;

    // Write the position state to the response buffer, written will be the length of the string
    int written = snprintf(response_buffer, sizeof(response_buffer), POSITION_FORMAT, target->position_state);

    // Send the response buffer back to the master with the null-terminator
    spi_write_blocking(SPI_PORT, (uint8_t*)response_buffer, written + 1);

    printf("Sent %c stepper position: %s\n", motor, response_buffer);
}

void handle_spi_command(uint8_t command) {
    switch (command) {
    case CALIBRATE: {
        printf("Calibration started...\n");
        spi_read_blocking(SPI_PORT, DUMMY, &command, 1);
        break;
    }

    case COMMAND_POSITION: {
        uint8_t motor;
        spi_read_blocking(SPI_PORT, DUMMY, &motor, 1);
        if (IS_VALID_MOTOR(motor))
            handle_position_command(motor);
        break;
    }

    case STATE_POSITION: {
        uint8_t motor;
        spi_read_blocking(SPI_PORT, DUMMY, &motor, 1);
        spi_sub_read_8_blocking(SPI_PORT);  // Read null-terminator byte
        if (IS_VALID_MOTOR(motor))
            handle_position_state_request(motor);
        break;
    }

    case NULL: {
        printf("Null-Terminator received.\n");
        break;
    }

    case DUMMY: {
        printf("Extra dummy byte received.\n");
        break;
    }

    default: {
        printf("Unknown command received: %x\n", command);
        break;
    }
    }
}

int main() {
    stdio_init_all();
    pico_led_init();
    spi_slave_init(SPI_PORT);
    sleep_ms(2000);  // Wait for serial monitor to open
    watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

    absolute_time_t last_position_update = get_absolute_time();
    absolute_time_t last_blink = get_absolute_time();

    bool blink_state = true;

    while (true) {
        // Handle the incoming SPI commands
        if (spi_is_readable(SPI_PORT)) {
            uint8_t command;
            spi_read_blocking(SPI_PORT, DUMMY, &command, 1);
            handle_spi_command(command);
            command = DUMMY;  // Reset the byte variable to a non-command reliant value
        }

        // Simulate moving the motors at a fixed frequency
        if (absolute_time_diff_us(last_position_update, get_absolute_time()) > POSITION_UPDATE_INTERVAL_US) {
            update_position(&l_stepper);
            update_position(&r_stepper);
            last_position_update = get_absolute_time();
        }

        // Blink LED just to know its alive
        if (absolute_time_diff_us(last_blink, get_absolute_time()) > BLINK_INTERVAL_US) {
            blink_state = !blink_state;
            pico_set_led(blink_state);
            last_blink = get_absolute_time();
        }

        // Pet the dog
        watchdog_update();
    }
}
