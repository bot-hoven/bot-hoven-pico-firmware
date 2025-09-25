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
#define SPI_BAUDRATE_HZ 125'000

#define STEP_PIN_L 10
#define DIR_PIN_L 11
#define STEP_PIN_R 6
#define DIR_PIN_R 7

#define CW 1
#define CCW 0

#define LL_LIMIT 2
#define LR_LIMIT 3
#define RR_LIMIT 5

// rail is ~88k steps with a 6400 steps/rev microstepping config
// therefore use 88k/1.2192 = 72k steps/meter as an initial guess for
// calibration
#define CALIB_STEPS_PER_METER 72'000
#define CALIB_SPEED_MPS 0.3f

#define MAX_VEL_MPS 1.0f
#define HAND_WIDTH 0.205 // meters
#define HOME_FROM_CENTER 0.25f

static const uint8_t LIMIT_PINS[] = {LL_LIMIT, LR_LIMIT, RR_LIMIT};

// 2^10 - 1
// at a step count of ~72k steps/meter, this wrap values gives
// a max speed of 1 meter/sec and a min speed of 3.9 millimeter/sec
static constexpr uint16_t PWM_WRAP = 2047;

volatile int64_t step_count_l = 0;
volatile int64_t step_count_r = 0;
bool dir_l_forward = true;
bool dir_r_forward = true;

uint pwm_slice_l, pwm_chan_l;
uint pwm_slice_r, pwm_chan_r;

// these get filled in by calibrate()
float steps_per_meter_l = 0.0f;
float steps_per_meter_r = 0.0f;

// during calibration, we don’t want the ISR to fault out
volatile bool is_calibrating = false;
volatile bool critical_fault = false;

// isr increments/decrements step counters on each PWM wrap
void __time_critical_func(on_pwm_wrap)() {
  uint32_t irq = pwm_get_irq_status_mask();
  for (uint slice = 0; slice < 8; ++slice) {
    if (irq & (1u << slice)) {
      pwm_clear_irq(slice);
      if (slice == pwm_slice_l)
        step_count_l += dir_l_forward ? +1 : -1;
      else if (slice == pwm_slice_r)
        step_count_r += dir_r_forward ? +1 : -1;
    }
  }
}

// stop motors if not calibrating
void limit_switch_handler(uint gpio, uint32_t events) {
  if (!is_calibrating) {
    pwm_set_enabled(pwm_slice_l, false);
    pwm_set_enabled(pwm_slice_r, false);
    critical_fault = true;
    gpio_put(PICO_DEFAULT_LED_PIN, 1);
  }
}

void init_limit_switches() {
  for (auto pin : LIMIT_PINS) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    gpio_pull_down(pin);
    gpio_set_irq_enabled_with_callback(pin, GPIO_IRQ_EDGE_RISE, true,
                                       limit_switch_handler);
  }
}

// set up step/DIR pins and PWM interrupts
void init_hardware() {
  // direction pins
  gpio_init(DIR_PIN_L);
  gpio_set_dir(DIR_PIN_L, GPIO_OUT);
  gpio_init(DIR_PIN_R);
  gpio_set_dir(DIR_PIN_R, GPIO_OUT);

  // step pins (PWM)
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

  irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
  irq_set_enabled(PWM_IRQ_WRAP, true);
  pwm_clear_irq(pwm_slice_l);
  pwm_clear_irq(pwm_slice_r);
  pwm_set_irq_enabled(pwm_slice_l, true);
  pwm_set_irq_enabled(pwm_slice_r, true);

  pwm_set_enabled(pwm_slice_l, false);
  pwm_set_enabled(pwm_slice_r, false);
}

// set a motor’s PWM for a given velocity (m/s)
void set_motor_pwm(uint slice, uint chan, uint8_t dir, float vel_mps) {
  if (slice == pwm_slice_l) {
    gpio_put(DIR_PIN_L, dir);
    dir_l_forward = (dir == CW);
  } else {
    gpio_put(DIR_PIN_R, dir);
    dir_r_forward = (dir == CW);
  }

  float steps_per_meter =
      (slice == pwm_slice_l ? steps_per_meter_l : steps_per_meter_r);
  float freq = fabsf(vel_mps) *
               (is_calibrating ? CALIB_STEPS_PER_METER : steps_per_meter);

  if (freq > 0.0f && !critical_fault) {
    float div = float(clock_get_hz(clk_sys)) / (freq * (PWM_WRAP + 1));
    div = std::max(0.0f, std::min(255.0f, div));
    pwm_set_clkdiv(slice, div);
    pwm_set_enabled(slice, true);
  } else {
    pwm_set_enabled(slice, false);
  }
}

// move until a particular limit switch is pressed
void move_until(uint slice, uint chan, uint8_t direction, float vel_mps,
                uint limit_pin) {
  set_motor_pwm(slice, chan, direction, vel_mps);
  while (!gpio_get(limit_pin)) {
    tight_loop_contents();
  }
  set_motor_pwm(slice, chan, direction, 0.0f);
  sleep_ms(50); // debounce
}

void calibrate() {
  is_calibrating = true;
  critical_fault = false;

  sleep_ms(100);

  // 1. bring left to LL, right to RR
  move_until(pwm_slice_l, pwm_chan_l, CCW, CALIB_SPEED_MPS, LL_LIMIT);
  move_until(pwm_slice_r, pwm_chan_r, CW, CALIB_SPEED_MPS, RR_LIMIT);

  // 2. measure left full travel
  step_count_l = 0;
  move_until(pwm_slice_l, pwm_chan_l, CW, CALIB_SPEED_MPS, LR_LIMIT);
  int64_t left_max = abs(step_count_l);

  // 3. return left to LL
  move_until(pwm_slice_l, pwm_chan_l, CCW, CALIB_SPEED_MPS, LL_LIMIT);

  // 4. measure right full travel
  step_count_r = 0;
  move_until(pwm_slice_r, pwm_chan_r, CCW, CALIB_SPEED_MPS, LR_LIMIT);
  int64_t right_max = abs(step_count_r);

  // 5. return right to RR
  move_until(pwm_slice_r, pwm_chan_r, CW, CALIB_SPEED_MPS, RR_LIMIT);

  // compute steps/meter
  const float rail_length = 1.2192f; // meters
  steps_per_meter_l = float(left_max) / (rail_length - HAND_WIDTH*2);
  steps_per_meter_r = float(right_max) / (rail_length - HAND_WIDTH*2);
  // printf("Calibrated!\nL: %f steps_per_meter, R: %f steps_per_meter\n",
  //       steps_per_meter_l, steps_per_meter_r);

  // 6. move to home offsets (+-0.3 m from center) and zero-counters
  const float half = rail_length / 2.0f;

  step_count_l = 0;
  uint32_t steps_l = uint32_t((half - HOME_FROM_CENTER - HAND_WIDTH/2) * steps_per_meter_l);
  set_motor_pwm(pwm_slice_l, pwm_chan_l, CW, CALIB_SPEED_MPS);
  while (uint64_t(step_count_l) < steps_l) {
    tight_loop_contents();
  }
  set_motor_pwm(pwm_slice_l, pwm_chan_l, 0.0f, 0.0f);
  step_count_l = 0; // zero position

  step_count_r = 0;
  int32_t steps_r = -1 * uint32_t((half - HOME_FROM_CENTER - HAND_WIDTH/2) * steps_per_meter_r);
  set_motor_pwm(pwm_slice_r, pwm_chan_r, CCW, CALIB_SPEED_MPS);
  while (step_count_r > steps_r) {
    tight_loop_contents();
  }

  set_motor_pwm(pwm_slice_r, pwm_chan_r, 0.0f, 0.0f);
  step_count_r = 0; // zero position

  is_calibrating = false;
}

int main() {
  stdio_init_all();
  sleep_ms(1000);

  // blink LED on startup
  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  gpio_put(PICO_DEFAULT_LED_PIN, 1);
  sleep_ms(500);
  gpio_put(PICO_DEFAULT_LED_PIN, 0);

  init_hardware();
  init_limit_switches();

  calibrate();

  sleep_ms(1000);

  SPIHandler spi(SPI_PORT, PIN_MISO, PIN_MOSI, PIN_SCK, PIN_CS,
                 SPI_BAUDRATE_HZ);
  spi.begin();

  while (true) {
    // process SPI in blocking manner
    // this is fine because pulse generation is offloaded to PWM modules
    uint8_t frame[6];
    spi.read_frame(frame, 6);
    char cmd = frame[0];
    char motor = frame[1];

    if (cmd != 's' && cmd != 'g') {
      continue;
    }

    if (cmd == 's') {
      float vel_mps;
      std::memcpy(&vel_mps, &frame[2], sizeof(vel_mps));

      if (std::isnan(vel_mps) || std::abs(vel_mps) > MAX_VEL_MPS) {
        continue;
      }

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
      float steps_per_meter =
          (motor == 'l' ? steps_per_meter_l : steps_per_meter_r);
      float pos_m = float(cnt) / steps_per_meter;
      std::memcpy(curr_pos, &pos_m, sizeof(pos_m));
      spi.write_response(curr_pos, 4);
    }
  }

  return 0;
}
