#include "soc/soc.h"
#include "soc/rtc_cntl_reg.h"

#include <TaskScheduler.h>
#include <TinyStepper_28BYJ_48.h>

#define CAMERA_MODEL_AI_THINKER // look in camera_pins.h to find the camera type you have
#include "camera_pins.h"        // Must include this after setting
#include "esp_camera.h"

#define FRAME_SIZE FRAMESIZE_QVGA // Frambuffer size (matches resolution below)
#define WIDTH 320                 // Resolution Width
#define HEIGHT 240                // Resolution height

#define STEPS_PER_DEGREE 6                 // How many degrees per block to turn 11.37
#define STEPS_PER_SECOND 4096              // How fast the stepper turns
#define ACCELLERATION_STEPS_PER_SECOND 1000
#define TOTAL_STEPS (STEPS_PER_DEGREE * 360)
#define FOV_DEGREES 78
#define FOV_TOTAL_STEPS (STEPS_PER_DEGREE * FOV_DEGREES)

#define RED_PIN (2)
#define BLUE_PIN (4)

#define MOTOR_IN1_PIN (12)
#define MOTOR_IN2_PIN (13)
#define MOTOR_IN3_PIN (15)
#define MOTOR_IN4_PIN (14)

#define NUMBER_OF_REGIONS (10)

#define MAX_INCREMENT_STEP (100)

#define LOG_FRAME (0)

typedef struct {
  uint32_t total_diff = 0;
  uint32_t region_diff[NUMBER_OF_REGIONS] = {0};
} frame_diff_t;

const uint32_t total_max_diff = HEIGHT * WIDTH * 256;
const uint32_t region_max_diff = (float)total_max_diff / (float)NUMBER_OF_REGIONS;

const float diff_threshold_procentage = 0.03;
const float total_diff_threshold = (float)total_max_diff * diff_threshold_procentage;
const float region_diff_threshold = (float)region_max_diff * diff_threshold_procentage;

camera_fb_t* current_fb = NULL;
camera_fb_t* previous_fb = NULL;

frame_diff_t m_frame_diff;

int target_stepper_position = 0;
int current_stepper_position = 0;

TinyStepper_28BYJ_48 stepper;

void moveStepper();
void incrementStepper();

// Scheduler
Scheduler scheduler;
Task stepperTask(100, TASK_FOREVER, &moveStepper, &scheduler, true);

static void setup_stepper()
{
  stepper.connectToPins(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
  stepper.setSpeedInStepsPerSecond(STEPS_PER_SECOND);
  stepper.setAccelerationInStepsPerSecondPerSecond(ACCELLERATION_STEPS_PER_SECOND);
}

static void setup_camera()
{
    camera_config_t config;

    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer = LEDC_TIMER_0;
    config.pin_d0 = Y2_GPIO_NUM;
    config.pin_d1 = Y3_GPIO_NUM;
    config.pin_d2 = Y4_GPIO_NUM;
    config.pin_d3 = Y5_GPIO_NUM;
    config.pin_d4 = Y6_GPIO_NUM;
    config.pin_d5 = Y7_GPIO_NUM;
    config.pin_d6 = Y8_GPIO_NUM;
    config.pin_d7 = Y9_GPIO_NUM;
    config.pin_xclk = XCLK_GPIO_NUM;
    config.pin_pclk = PCLK_GPIO_NUM;
    config.pin_vsync = VSYNC_GPIO_NUM;
    config.pin_href = HREF_GPIO_NUM;
    config.pin_sscb_sda = SIOD_GPIO_NUM;
    config.pin_sscb_scl = SIOC_GPIO_NUM;
    config.pin_pwdn = PWDN_GPIO_NUM;
    config.pin_reset = RESET_GPIO_NUM;
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_GRAYSCALE;
    config.frame_size = FRAME_SIZE;
    config.jpeg_quality = 12;
    config.fb_count = 2;

    esp_camera_deinit();

    esp_err_t err = esp_camera_init(&config);
    if (err != ESP_OK) {
      Serial.printf("Camera init failed with error 0x%x", err);
      exit(1);
    }

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_framesize(sensor, FRAME_SIZE);
    sensor->set_brightness(sensor, 2);
    sensor->set_contrast(sensor, 2);
    sensor->set_special_effect(sensor, 2);
}

static void setup_led()
{
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);
  //digitalWrite(BLUE_PIN, HIGH);
}

static void start_red_light()
{
  // Serial.println("Enter start_red_light");
  digitalWrite(BLUE_PIN, LOW);
  digitalWrite(RED_PIN, HIGH);
  // Serial.println("Exit start_red_light");
}

static void stop_red_light()
{
  // Serial.println("Enter stop_red_light");
  digitalWrite(BLUE_PIN, HIGH);
  digitalWrite(RED_PIN, LOW);
  // Serial.println("Exit stop_red_light");
}

void incrementStepper()
{
  int diff = abs(current_stepper_position - target_stepper_position);
  int incrementPosition = 0;
  
  if(diff > 0) {
    start_red_light();
    if(diff < MAX_INCREMENT_STEP) {
      incrementPosition = target_stepper_position;
    }
    else if (current_stepper_position > target_stepper_position) {
      incrementPosition = current_stepper_position - MAX_INCREMENT_STEP;
    }
    else {
      incrementPosition = current_stepper_position + MAX_INCREMENT_STEP;
    }
    stepper.moveToPositionInSteps(incrementPosition);
    current_stepper_position = incrementPosition;
  } else {
    stop_red_light();
  }
}

void moveStepper()
{
  // Serial.println("Enter moveStepper");
  int diff = abs(current_stepper_position - target_stepper_position);
  
  if(diff > 0) {
    // start_red_light();
    stepper.moveToPositionInSteps(target_stepper_position);
    // stop_red_light();
    current_stepper_position = target_stepper_position;
  }
  // Serial.println("Exit moveStepper");
}

static bool update_frames()
{
  // Serial.println("Enter update_frames");

  if(previous_fb) {
    // Serial.println("Returning frame buffer");
    esp_camera_fb_return(previous_fb);
  }

  previous_fb = current_fb;

  camera_fb_t *frame_buffer = esp_camera_fb_get();
  if(!frame_buffer) {
    // Serial.println("Can't get frame buffer");
    return false;
  }

  current_fb = frame_buffer;

  // Serial.println("Exit update_frames");

  return current_fb && previous_fb;
}

static void calculate_frame_diff()
{
  // Serial.println("Enter calculate_frame_diff");
  static int pic_width = (int)current_fb->width;
  static int pic_height = (int)current_fb->height;
  static float region_width_factor = (float)NUMBER_OF_REGIONS / (float)pic_width;

  uint8_t* previous_buff = previous_fb->buf;
  uint8_t* current_buff = current_fb->buf;

  memset(&m_frame_diff, 0, sizeof(frame_diff_t));
  for(int h = 0; h < pic_height; h++) {
    for(int w = 0; w < pic_width; w++) {
      uint8_t region = floor((float)w * region_width_factor);
      uint32_t index = (h * pic_width) + w;
      uint8_t previous_pixel = previous_buff[index];
      uint8_t current_pixel = current_buff[index];
      uint8_t pixel_diff = abs(previous_pixel - current_pixel);
      m_frame_diff.total_diff += pixel_diff;
      m_frame_diff.region_diff[region] += pixel_diff;
    }
  }
}

static uint8_t get_highest_diff_region()
{
  // Serial.println("Enter get_highest_diff_region");
  uint32_t current_highest_region_value = 0;
  uint8_t current_highest_region = 0;
  for(int r = 0; r < NUMBER_OF_REGIONS; r++) {
    uint32_t region_diff = m_frame_diff.region_diff[r];
    if(region_diff > current_highest_region_value) {
      current_highest_region = r;
      current_highest_region_value = region_diff;
    }
  }
  // Serial.println("Exit get_highest_diff_region");
  return current_highest_region;
}

static bool frame_diff_passes_threshold()
{
  // Serial.println("Enter frame_diff_passes_threshold");

  bool total_threshold_passed = m_frame_diff.total_diff > total_diff_threshold;
  bool region_threshold_passed = false;
  if(total_threshold_passed) {
    for(int i = 0; i < NUMBER_OF_REGIONS; i++) {
      if(m_frame_diff.region_diff[i] > region_diff_threshold) {
        region_threshold_passed = true;
        break;
      }
    }
  }

  // Serial.println("Exit frame_diff_passes_threshold");
  return total_threshold_passed && region_threshold_passed;
}

static void set_target_stepper_position(uint8_t highest_diff_region)
{
  // Serial.println("Enter set_target_stepper_position");
  target_stepper_position = highest_diff_region * (FOV_TOTAL_STEPS / NUMBER_OF_REGIONS);
  // Serial.println("Exit set_target_stepper_position");
}


static void log_frame()
{
  float total_diff_percentage = ((float)m_frame_diff.total_diff / (float)total_max_diff) * 100;
  Serial.printf("Total Diff: %.2f%%\n", total_diff_percentage);
  for(int i = 0; i < NUMBER_OF_REGIONS; i++) {
    float region_diff_percentage = ((float)m_frame_diff.region_diff[i] / (float)region_max_diff) * 100;
    Serial.printf("Region %d Diff: %.2f%%\n",i , region_diff_percentage);
  }
}


void setup()
{
    WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    Serial.begin(115200);
    Serial.println("Begin Setup...");

    Serial.printf("total_diff_threshold: %.2f\n", total_diff_threshold);
    Serial.printf("region_diff_threshold: %.2f\n", region_diff_threshold);

    setup_camera();
    setup_stepper();
    setup_led();

    scheduler.startNow();

    Serial.println("End Setup...");
}

void loop()
{
  // Serial.println("Enter loop");
  bool success = update_frames();

  if(success) {
    calculate_frame_diff();
    if(frame_diff_passes_threshold()) {
      if(LOG_FRAME) {
        log_frame();
      }
      uint8_t highest_diff_region = get_highest_diff_region();
      // Serial.printf("highest_diff_region:%d\n", highest_diff_region);
      set_target_stepper_position(highest_diff_region);
    }
  }

  scheduler.execute();

  // Serial.println("Exit loop");
}