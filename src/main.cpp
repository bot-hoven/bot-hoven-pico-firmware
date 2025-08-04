#include <cmath>
#include <cstdio>
#include <cstring>

#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/pwm.h"
#include "hardware/spi.h"
#include "pico/stdlib.h"

#include "include/SPIHandler.h"

#define PIN_MISO 16
#define PIN_CS 17
#define PIN_SCK 18
#define PIN_MOSI 19
#define SPI_PORT spi0
#define SPI_BAUDRATE_HZ 500000 // 500 kHz

#define DIR_PIN_L 2
#define STEP_PIN_L 3
#define DIR_PIN_R 4
#define STEP_PIN_R 5

#define MAX_VEL_MPS 10.0f

static const uint8_t LIMIT_PINS[] = {2, 3, 4, 5};

static constexpr float STEPS_PER_METER = 10000.0f;
static constexpr uint16_t PWM_WRAP = 255; // use 8-bit counter

volatile int64_t step_count_l = 0;
volatile int64_t step_count_r = 0;
bool dir_l_forward = true;
bool dir_r_forward = true;
uint pwm_slice_l, pwm_chan_l;
uint pwm_slice_r, pwm_chan_r;
volatile bool critical_fault = false;

// pwm-wrap IRQ: bump the step counters (one wrap == one pulse period)
void __time_critical_func(on_pwm_wrap)() {
  uint32_t irq = pwm_get_irq_status_mask();
  for (uint slice = 0; slice < 8; ++slice) {
    if (irq & (1u << slice)) {
      pwm_clear_irq(slice);
      if (slice == pwm_slice_l)
        step_count_l += dir_l_forward ? 1 : -1;
      else if (slice == pwm_slice_r)
        step_count_r += dir_r_forward ? 1 : -1;
    }
  }
}

void limit_switch_handler(uint gpio, uint32_t events) {
  printf("LIMIT SWITCH PRESSED");

  // stop everything immediately
  pwm_set_enabled(pwm_slice_l, false);
  pwm_set_enabled(pwm_slice_r, false);
  critical_fault = true;

  // blink onboard LED to indicate fault
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
}

// set up all limit-switch GPIOs with same interrupt callback
void init_limit_switches() {
  for (auto pin : LIMIT_PINS) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_down(pin);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE, true,
                                       limit_switch_handler);
  }
}

// adjust one motor’s PWM for a given velocity
void set_motor_pwm(uint slice, uint chan, bool dir_forward, float vel_mps) {
  gpio_put((slice == pwm_slice_l ? DIR_PIN_L : DIR_PIN_R), dir_forward);
  float freq = fabsf(vel_mps) * STEPS_PER_METER;
  if (freq > 0.0f && !critical_fault) {
    float div = float(clock_get_hz(clk_sys)) / (freq * (PWM_WRAP + 1));
    pwm_set_clkdiv(slice, div);
    pwm_set_enabled(slice, true);
  } else {
    pwm_set_enabled(slice, false);
  }
}

// setup and SPI handling
int main() {
  stdio_init_all();
  sleep_ms(200);

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(200);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);

  gpio_init(DIR_PIN_L);
  gpio_set_dir(DIR_PIN_L, GPIO_OUT);
  gpio_init(DIR_PIN_R);
  gpio_set_dir(DIR_PIN_R, GPIO_OUT);

  // configure pico pwm
  gpio_set_function(STEP_PIN_L, GPIO_FUNC_PWM);
  gpio_set_function(STEP_PIN_R, GPIO_FUNC_PWM);
  pwm_slice_l = pwm_gpio_to_slice_num(STEP_PIN_L);
  pwm_chan_l = pwm_gpio_to_channel(STEP_PIN_L);
  pwm_slice_r = pwm_gpio_to_slice_num(STEP_PIN_R);
  pwm_chan_r = pwm_gpio_to_channel(STEP_PIN_R);
  pwm_set_wrap(pwm_slice_l, PWM_WRAP);
  pwm_set_wrap(pwm_slice_r, PWM_WRAP);
  pwm_set_chan_level(pwm_slice_l, pwm_chan_l, PWM_WRAP / 2);
  pwm_set_chan_level(pwm_slice_r, pwm_chan_r, PWM_WRAP / 2);

  // configure pwm IRQ on wrap
  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_clear_irq(pwm_slice_l);
  pwm_clear_irq(pwm_slice_r);
  pwm_set_irq_enabled(pwm_slice_l, true);
  pwm_set_irq_enabled(pwm_slice_r, true);
  pwm_set_enabled(pwm_slice_l, false);
  pwm_set_enabled(pwm_slice_r, false);

  init_limit_switches();

  SPIHandler spi(SPI_PORT, PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CS,
                 SPI_BAUDRATE_HZ);
  spi.begin();

  while (true) {
    uint8_t frame[6];
    spi.read_frame(frame, 6);
    char cmd = frame[0];
    char motor = frame[1];

    if (cmd != 's' && cmd != 'g') {
      continue;
    }

    printf("Received command: %c%c\n", cmd, motor);

    if (cmd == 's') {
      float vel_mps;
      std::memcpy(&vel_mps, &frame[2], sizeof(vel_mps));

      if (std::isnan(vel_mps) || std::abs(vel_mps) > MAX_VEL_MPS) {
        continue;
      }

      printf("Received set value: %f\n", vel_mps);

      if (motor == 'l') {
        dir_l_forward = (vel_mps >= 0);
        set_motor_pwm(pwm_slice_l, pwm_chan_l, dir_l_forward, vel_mps);
      } else {
        dir_r_forward = (vel_mps >= 0);
        set_motor_pwm(pwm_slice_r, pwm_chan_r, dir_r_forward, vel_mps);
      }
    } else if (cmd == 'g') {
      uint8_t curr_pos[4];
      int64_t cnt = (motor == 'l' ? step_count_l : step_count_r);
      float pos_m = float(cnt) / STEPS_PER_METER;
      pos_m = 17.38;
      std::memcpy(curr_pos, &pos_m, sizeof(pos_m));
      spi.write_response(curr_pos, 4);
      printf("Sent back: %f\n", pos_m);
    }
  }

  return 0;
}
