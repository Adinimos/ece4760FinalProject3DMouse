/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 */

#include <pico/stdio.h>
#include <stdlib.h>
#include <stdio.h>
#include "pico/stdlib.h"
#include <string.h>

//#include "bsp/board_api.h"
#include "tusb.h"

#include "usb_descriptors.h"
#include "hardware/gpio.h"

// Include standard libraries
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/multicore.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
// Include custom libraries
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1_4.h"
#include "quat_math.c"

// Arrays in which raw measurements will be stored
fix15 acceleration[3], gyro[3];

// semaphore
static struct pt_sem imu_sem ;

// Some paramters for PWM
#define WRAPVAL 5000
#define CLKDIV  25.0
uint slice_num ;

#define BUTTON_PIN 12
#define BUTTON_PIN_CLICK 5

// Debounce settings for click button (in 10ms increments since loop runs at 100Hz)
#define DEBOUNCE_COUNT 3  // Require 3 consecutive same readings (~30ms)

//const float dt = 1.0f / (150000000.0f / (CLKDIV * (WRAPVAL + 1)));
const float DEG2RAD = (float)3.14159265358979323846f / 180.0f;
const float RAD2DEG = 180.0f / (float)3.14159265358979323846f;

vect_t f_body = {0.0f, 1.0f, 0.0f};

volatile float gx, gy, gz;
volatile float ax, ay, az;
volatile float x_mouse_pos = 0.0f;
volatile float y_mouse_pos = 0.0f;

/* Blink pattern
 * - 250 ms  : device not mounted
 * - 1000 ms : device mounted
 * - 2500 ms : device is suspended
 */
enum  {
  BLINK_NOT_MOUNTED = 250,
  BLINK_MOUNTED = 1000,
  BLINK_SUSPENDED = 2500,
};

static uint32_t blink_interval_ms = BLINK_NOT_MOUNTED;
const float dt = (CLKDIV * (WRAPVAL + 1)) / 150000000.0f;

void led_blinking_task(void);
void hid_task(void);

// -------------------------------------------------------
// Interrupt service routine
void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    // Read the IMU
    // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
    // If you want these values in floating point, call fix2float15() on
    // the raw measurements.
    mpu6050_read_raw(acceleration, gyro);

    gx = fix2float15(gyro[0]) * DEG2RAD;
    gy = fix2float15(gyro[1]) * DEG2RAD;
    gz = fix2float15(gyro[2]) * DEG2RAD;
    
    ax = fix2float15(acceleration[0]);
    ay = fix2float15(acceleration[1]);
    az = fix2float15(acceleration[2]);

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &imu_sem);
}

// Thread that updates the quanternion from IMU data
static PT_THREAD (protothread_imu(struct pt *pt))
{
    static absolute_time_t last_time;

    // Indicate start of thread
    PT_BEGIN(pt) ;


    last_time = get_absolute_time();

    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &imu_sem);

        absolute_time_t now = get_absolute_time();
        int64_t dt_us = absolute_time_diff_us(last_time, now);
        last_time = now;

        float dt = dt_us * 1e-6f;  // seconds

        imu_update(gx, gy, gz, ax, ay, az, dt);



        //imu_get_euler(&yaw, &pitch, &roll);

        //vect_t f_world = quat_rotate_vec(imu_get_quaternion(), f_body);

        //printf("YPR(deg)=%.1f,%.1f,%.1f  pointer=(%.3f,%.3f,%.3f)\n",
        //   yaw*RAD2DEG, pitch*RAD2DEG, roll*RAD2DEG,
        //   f_world.x, f_world.y, f_world.z);
        //quat_t q = imu_get_quaternion();
        
        //printf("%f,%f,%f,%f\n", q.w, q.x, q.y, q.z);

        PT_YIELD(pt); //TODO: needed?
    }

    // Indicate end of thread
    PT_END(pt);
}

// Thread that prints to Serial for web analysis
// Runs at 100Hz
static PT_THREAD (protothread_print(struct pt *pt)) {

    // Indicate start of thread
    PT_BEGIN(pt) ;

    while (true) {
      //tud_task();
      quat_t q = imu_get_quaternion();
        
      //Uncomment to print for rotating bunny: "https://adafruit.github.io/Adafruit_WebSerial_3DModelViewer/"
      //printf("Quaternion: %.5f, %.5f, %.5f, %.5f\n", q.w, q.x, q.y, q.z);

      //Uncomment to print for CAD viewer
      printf("%f %f %f %f\n", q.w, q.x, q.y, q.z);
      PT_YIELD_usec(10000); // yield for 10ms
    }

    // Indicate end of thread
    PT_END(pt);
}

// Thread to turn on and off built-in led (helps us see that the program is working)
static PT_THREAD (protothread_blinkey(struct pt *pt))
{
    PT_BEGIN(pt);
    while (1) {
        gpio_put(25, 1);
        PT_YIELD_usec(500000); // wait 0.5 seconds
        gpio_put(25, 0);
        PT_YIELD_usec(500000); // wait 0.5 seconds
    }
    PT_END(pt);
}

bool previous_button_state = false;
bool mouse_state = true;

// Thread that controls mouse with IMU using stabilized projection
static PT_THREAD (protothread_mouse(struct pt *pt)) {

    PT_BEGIN(pt);

    static quat_t q_center;
    static bool center_set = false;
    
    // Previous screen position
    static float prev_screen_x = 0.0f;
    static float prev_screen_y = 0.0f;
    
    // Smoothing buffers
    static float smooth_x = 0.0f;
    static float smooth_y = 0.0f;
    
    // Debounce state for click button
    static uint8_t debounce_counter = 0;
    static bool debounced_click_state = false;

    while (true) {
        tud_task();
        
        quat_t q_current = imu_get_quaternion();

        // Button state
        bool is_button_down = !gpio_get(BUTTON_PIN);
        bool raw_click_state = !gpio_get(BUTTON_PIN_CLICK);

        if (!center_set) {
            q_center = q_current;
            center_set = true;
            prev_screen_x = 0.0f;
            prev_screen_y = 0.0f;
            smooth_x = 0.0f;
            smooth_y = 0.0f;
        }

        // FREEZE MODE: When button is held, cursor doesn't move
        // This lets you physically recenter the IMU
        if (is_button_down) {
            // Update the center orientation to current position
            q_center = q_current;

            // Reset smoothing buffers to prevent jumps when releasing
            smooth_x = 0.0f;
            smooth_y = 0.0f;
            prev_screen_x = 0.0f;
            prev_screen_y = 0.0f;
            
            // Don't send any mouse movement
            if (tud_hid_ready() && mouse_state) {
                tud_hid_mouse_report(REPORT_ID_MOUSE, 0, 0, 0, 0, 0);
            }
            
        } else {
            // NORMAL MODE: Use IMU orientation to control cursor

            // Calculate orientation relative to center
            quat_t q_center_inv = quat_conj(q_center);
            quat_t q_relative = quat_mul(q_center_inv, q_current);
            q_relative = quat_normalize(q_relative);

            // Get pointing direction
            vect_t pointing_dir = quat_rotate_vec(q_relative, f_body);

            // Calculate angles (more stable than ray-casting)
            float horizontal_angle = atan2f(pointing_dir.x, pointing_dir.y);
            
            float horizontal_dist = sqrtf(pointing_dir.x * pointing_dir.x + 
                                          pointing_dir.y * pointing_dir.y);
            float vertical_angle = atan2f(pointing_dir.z, horizontal_dist);
            
            // Convert angles to screen position
            const float horizontal_sensitivity = 2000.0f;  // Increased from 800
            const float vertical_sensitivity = 2000.0f;    // Increased from 800
            
            float cursor_x = horizontal_angle * horizontal_sensitivity;
            float cursor_y = -vertical_angle * vertical_sensitivity;
            
            // Reduced smoothing for faster response
            const float angle_smoothing = 0.8f;  // Reduced from 0.7
            smooth_x = smooth_x * angle_smoothing + cursor_x * (1.0f - angle_smoothing);
            smooth_y = smooth_y * angle_smoothing + cursor_y * (1.0f - angle_smoothing);
            
            const float position_smoothing = 0.8f;  // Reduced from 0.3
            float final_x = prev_screen_x * position_smoothing + smooth_x * (1.0f - position_smoothing);
            float final_y = prev_screen_y * position_smoothing + smooth_y * (1.0f - position_smoothing);
            
            // Calculate delta
            float dx_float = final_x - prev_screen_x;
            float dy_float = final_y - prev_screen_y;
            
            // Update previous position
            prev_screen_x = final_x;
            prev_screen_y = final_y;
            
            // Clamp to HID range
            int8_t dx = (int8_t)fmaxf(-127.0f, fminf(127.0f, dx_float));
            int8_t dy = (int8_t)fmaxf(-127.0f, fminf(127.0f, dy_float));

            // Debounce the click button
            if (raw_click_state == debounced_click_state) {
                // Button stable, reset counter
                debounce_counter = 0;
            } else {
                // Button changed, increment counter
                debounce_counter++;
                if (debounce_counter >= DEBOUNCE_COUNT) {
                    // Stable for long enough, accept the new state
                    debounced_click_state = raw_click_state;
                    debounce_counter = 0;
                }
            }
            bool is_button_down_click = debounced_click_state;

            //debounce button press for click
            int8_t mouse_mask = is_button_down_click ? MOUSE_BUTTON_LEFT : 0; // Use LEFT_BUTTON
            
            // Send mouse report (no button press in this mode)
            if (tud_hid_ready() && mouse_state) {
                tud_hid_mouse_report(REPORT_ID_MOUSE, mouse_mask, dx, dy, 0, 0);
            }
        }
        
        // Run at 100Hz
        PT_YIELD_usec(10000);
    }

    PT_END(pt);
}

// Entry point for core 1
void core1_entry() {
    pt_add_thread(protothread_imu) ;
    pt_schedule_start ;
}


/*------------- MAIN -------------*/
int main(void)
{
  // Overclock
  set_sys_clock_khz(150000, true) ;
  
  stdio_init_all();

  //initialize imu filter
  imu_filter_reset();

  ////////////////////////////////////////////////////////////////////////
    ///////////////////////// I2C CONFIGURATION ////////////////////////////
    i2c_init(I2C_CHAN, I2C_BAUD_RATE) ;
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C) ;
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C) ;

    // Pullup resistors on breakout board, don't need to turn on internals
    // gpio_pull_up(SDA_PIN) ;
    // gpio_pull_up(SCL_PIN) ;

    // MPU6050 initialization
    mpu6050_reset();
    mpu6050_read_raw(acceleration, gyro);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);

    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


  // init device stack on configured roothub port
  tud_init(0);

  // if (board_init_after_tusb) {
  //   board_init_after_tusb();
  // }

  //initialize button input on GPIO 12
  // Button setup
  gpio_init(BUTTON_PIN);
  gpio_set_dir(BUTTON_PIN, GPIO_IN);
  gpio_pull_up(BUTTON_PIN); // enable pull-up resistor so this is active high

  gpio_init(BUTTON_PIN_CLICK);
  gpio_set_dir(BUTTON_PIN_CLICK, GPIO_IN);
  gpio_pull_up(BUTTON_PIN_CLICK); // enable pull-up resistor so this is active high

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);

  PT_SEM_INIT(&imu_sem, 0); // initialize semaphore

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    //pt_add_thread(protothread_print) ;
    pt_add_thread(protothread_blinkey) ;
    pt_add_thread(protothread_mouse) ;
    pt_schedule_start ;  

  // while (1)
  // {
  //   tud_task(); // tinyusb device task
  //   led_blinking_task();

  //   hid_task();
  //   printf("Angle X: %.2f  Angle Y: %.2f  Angle Z: %.2f\n", angleX, angleY, angleZ);

  // }
}

//--------------------------------------------------------------------+
// Device callbacks
//--------------------------------------------------------------------+

// Invoked when device is mounted
void tud_mount_cb(void)
{
  blink_interval_ms = BLINK_MOUNTED;
}

// Invoked when device is unmounted
void tud_umount_cb(void)
{
  blink_interval_ms = BLINK_NOT_MOUNTED;
}

// Invoked when usb bus is suspended
// remote_wakeup_en : if host allow us  to perform remote wakeup
// Within 7ms, device must draw an average of current less than 2.5 mA from bus
void tud_suspend_cb(bool remote_wakeup_en)
{
  (void) remote_wakeup_en;
  blink_interval_ms = BLINK_SUSPENDED;
}

// Invoked when usb bus is resumed
void tud_resume_cb(void)
{
  blink_interval_ms = tud_mounted() ? BLINK_MOUNTED : BLINK_NOT_MOUNTED;
}

//--------------------------------------------------------------------+
// USB HID
//--------------------------------------------------------------------+

// static void send_hid_report(uint8_t report_id, uint32_t btn)
// {
//   // skip if hid is not ready yet
//   if ( !tud_hid_ready() ) return;
//   switch(report_id)
//   {
//     case REPORT_ID_KEYBOARD:
//     {
//       // use to avoid send multiple consecutive zero report for keyboard
//       static bool has_keyboard_key = false;

//       if ( btn )
//       {
//         uint8_t keycode[6] = { 0 };
//         keycode[0] = HID_KEY_A;

//         tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, keycode);
//         has_keyboard_key = true;
//       }else
//       {
//         // send empty key report if previously has key pressed
//         if (has_keyboard_key) tud_hid_keyboard_report(REPORT_ID_KEYBOARD, 0, NULL);
//         has_keyboard_key = false;
//       }
//     }
//     break;

//     case REPORT_ID_MOUSE:
//     {
//       int8_t const delta = 5;

//       // no button, right + down, no scroll, no pan
//       tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta, delta, 0, 0);
//     }
//     break;

//     case REPORT_ID_CONSUMER_CONTROL:
//     {
//       // use to avoid send multiple consecutive zero report
//       static bool has_consumer_key = false;

//       if ( btn )
//       {
//         // volume down
//         uint16_t volume_down = HID_USAGE_CONSUMER_VOLUME_DECREMENT;
//         tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &volume_down, 2);
//         has_consumer_key = true;
//       }else
//       {
//         // send empty key report (release key) if previously has key pressed
//         uint16_t empty_key = 0;
//         if (has_consumer_key) tud_hid_report(REPORT_ID_CONSUMER_CONTROL, &empty_key, 2);
//         has_consumer_key = false;
//       }
//     }
//     break;

//     case REPORT_ID_GAMEPAD:
//     {
//       // use to avoid send multiple consecutive zero report for keyboard
//       static bool has_gamepad_key = false;

//       hid_gamepad_report_t report =
//       {
//         .x   = 0, .y = 0, .z = 0, .rz = 0, .rx = 0, .ry = 0,
//         .hat = 0, .buttons = 0
//       };

//       if ( btn )
//       {
//         report.hat = GAMEPAD_HAT_UP;
//         report.buttons = GAMEPAD_BUTTON_A;
//         tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));

//         has_gamepad_key = true;
//       }else
//       {
//         report.hat = GAMEPAD_HAT_CENTERED;
//         report.buttons = 0;
//         if (has_gamepad_key) tud_hid_report(REPORT_ID_GAMEPAD, &report, sizeof(report));
//         has_gamepad_key = false;
//       }
//     }
//     break;

//     default: break;
//   }
// }

// bool previous_button_state = false;

// void hid_task(void) {
    
//     // Determine button state
//     bool is_button_down = !gpio_get(BUTTON_PIN); 

//     // Check if device is mounted
//     if ( tud_hid_ready() ) {
//         // Only send a mouse report if the button state changed
//         if (is_button_down != previous_button_state) {
//             //uint8_t button_mask = is_button_down ? MOUSE_BUTTON_LEFT : 0; // Use LEFT_BUTTON
//             int8_t const delta = 5;

//             // Mouse report: buttons, x, y, wheel, pan
//             tud_hid_mouse_report(REPORT_ID_MOUSE, 0x00, delta, delta, 0, 0);
//         }
//     }
//     previous_button_state = is_button_down;
// }

// Every 10ms, we will sent 1 report for each HID profile (keyboard, mouse etc ..)
// tud_hid_report_complete_cb() is used to send the next report after previous one is complete
// void hid_task(void)
// {
//   // Poll every 10ms
//   const uint32_t interval_ms = 10;
//   static uint32_t start_ms = 0;

//   if ( board_millis() - start_ms < interval_ms) return; // not enough time
//   start_ms += interval_ms;

//   uint32_t const btn = gpio_get(15); //board_button_read();

//   // Remote wakeup
//   if ( tud_suspended() && btn )
//   {
//     // Wake up host if we are in suspend mode
//     // and REMOTE_WAKEUP feature is enabled by host
//     tud_remote_wakeup();
//   }else
//   {
//     // Send the 1st of report chain, the rest will be sent by tud_hid_report_complete_cb()
//     send_hid_report(REPORT_ID_KEYBOARD, btn);
//   }
// }

// Invoked when sent REPORT successfully to host
// Application can use this to send the next report
// Note: For composite reports, report[0] is report ID
void tud_hid_report_complete_cb(uint8_t instance, uint8_t const* report, uint16_t len)
{
  (void) instance;
  (void) len;

  // uint8_t next_report_id = report[0] + 1u;

  // if (next_report_id < REPORT_ID_COUNT)
  // {
  //   send_hid_report(next_report_id, !gpio_get(BUTTON_PIN)); //board_button_read());
  // }
}

// Invoked when received GET_REPORT control request
// Application must fill buffer report's content and return its length.
// Return zero will cause the stack to STALL request
uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t* buffer, uint16_t reqlen)
{
  // TODO not Implemented
  (void) instance;
  (void) report_id;
  (void) report_type;
  (void) buffer;
  (void) reqlen;

  return 0;
}

// Invoked when received SET_REPORT control request or
// received data on OUT endpoint ( Report ID = 0, Type = 0 )
void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id, hid_report_type_t report_type, uint8_t const* buffer, uint16_t bufsize)
{
  (void) instance;

  if (report_type == HID_REPORT_TYPE_OUTPUT)
  {
    // Set keyboard LED e.g Capslock, Numlock etc...
    if (report_id == REPORT_ID_KEYBOARD)
    {
      // bufsize should be (at least) 1
      if ( bufsize < 1 ) return;

      uint8_t const kbd_leds = buffer[0];

      if (kbd_leds & KEYBOARD_LED_CAPSLOCK)
      {
        // Capslock On: disable blink, turn led on
        blink_interval_ms = 0;
        gpio_put(25, 0);
      }else
      {
        // Caplocks Off: back to normal blink
        gpio_put(25, 1);
        blink_interval_ms = BLINK_MOUNTED;
      }
    }
  }
}

//--------------------------------------------------------------------+
// BLINKING TASK
//--------------------------------------------------------------------+
// void led_blinking_task(void)
// {
//   static uint32_t start_ms = 0;
//   static bool led_state = false;

//   // blink is disabled
//   if (!blink_interval_ms) return;

//   // Blink every interval ms
//   if ( to_ms_since_boot(get_absolute_time()) - start_ms < blink_interval_ms) return; // not enough time
//   start_ms += blink_interval_ms;

//   my_led_write(led_state);
//   led_state = 1 - led_state; // toggle
// }

