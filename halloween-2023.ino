/**
   ESP32-CAM Directional Motion Detection (https://www.youtube.com/watch?v=NIbiG6at01g)
   Learn more on Makers Mashup - https://youtube.com/MakersMashup
   Licensed under the Attribution-ShareAlike 4.0 International (CC BY-SA 4.0)
   http://creativecommons.org/licenses/by-sa/4.0/

   PLEASE NOTE YOU WILL NEED TO ADD THE TaskSchedule and TinySTepper libarires to 
   run this code unaltered.  Use TOOLS->Manage Libraries in the arduino IDE.
   Also use the board manager and make sure you select ESP32 as an available
   board. The one I used specifically was https://amzn.to/3m2dJrl
   
*/
#include <TaskScheduler.h>
#include <TinyStepper_28BYJ_48.h>

#define CAMERA_MODEL_AI_THINKER // look in camera_pins.h to find the camera type you have
#include "camera_pins.h"        // Must include this after setting
#include "esp_camera.h"
#include "FS.h"     // SD Card ESP32
#include "SD_MMC.h" // SD Card ESP32

#define FRAME_SIZE FRAMESIZE_QVGA // Frambuffer size (matches resolution below)
#define WIDTH 320                 // Resolution Width
#define HEIGHT 240                // Resolution height
#define BLOCK_SIZE 4              // Size of each sensor block on the display (reduced granularity for speed)
#define W (WIDTH / BLOCK_SIZE)
#define H (HEIGHT / BLOCK_SIZE)
#define BLOCK_DIFF_THRESHOLD 1.5
#define IMAGE_DIFF_THRESHOLD 0.1
#define DEBUG 0                            // Good for making changes
#define INFO 1                             // Good to see whats happening (turn off to save CPU)
#define VIEWPORT_PIXELS WIDTH / BLOCK_SIZE // 320/4 = 80 Positions to look at in left to right view plane [-------X------] X is where motion exists
#define STEPS_PER_DEGREE 6                 // How many degrees per block to turn 11.37
#define STEPS_PER_SECOND 4096              // How fast the stepper turns
#define TOTAL_STEPS (STEPS_PER_DEGREE * 360)
#define FOV_DEGREES 78
#define FOV_TOTAL_STEPS (STEPS_PER_DEGREE * FOV_DEGREES)

#define ACCELLERATION_STEPS_PER_SECOND 1000 // How quickly the movement accellerates
#define FLASH_PIN 4                        // Pin of ESP32 Flash (Led)
#define FLASH_MODE 0                       // 0 = 0ff , 1 = flash, 2 = steady

uint16_t prev_frame[H][W] = {0};
uint16_t current_frame[H][W] = {0};
uint16_t empty_frame[H][W] = {0};
long motionView[VIEWPORT_PIXELS];

// stepper
const uint8_t MOTOR_IN1_PIN = 12;
const uint8_t MOTOR_IN2_PIN = 13;
const uint8_t MOTOR_IN3_PIN = 15;
const uint8_t MOTOR_IN4_PIN = 14;
int targetPos = 0;
int currentPos = 0;

TinyStepper_28BYJ_48 stepper;

// Scheduler
Scheduler tasks;
void moveStepper();
void incrementStepper();

Task tmvStepper(100, TASK_FOREVER, &moveStepper, &tasks, true);

#define MAX_INCREMENT_STEP (100)

void incrementStepper()
{
  int diff = abs(currentPos - targetPos);
  int incrementPosition = 0;
  
  if(diff > 0) {
    if(diff < MAX_INCREMENT_STEP) {
      incrementPosition = targetPos;
    }
    else if (currentPos > targetPos) { 
      incrementPosition = currentPos - MAX_INCREMENT_STEP;
    }
    else {
      incrementPosition = currentPos + MAX_INCREMENT_STEP;      
    }
    stepper.moveToPositionInSteps(incrementPosition);      
    currentPos = incrementPosition;
  }
}

void moveStepper()
{
  int diff = abs(currentPos - targetPos);
  int incrementPosition = 0;
  
  if(diff > 0) {  
    stepper.moveToPositionInSteps(targetPos);      
    currentPos = targetPos;
  }
}

long viewPortToRegion(long mv[])
{
    int maxVal = 0;
    int region = 0;
    int tmpVal = 0;
    char str_tmp[9];
    // Fill each char arry with the 8 bits of the 10 regions
    for (int i = 0; i < 10; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            str_tmp[j] = (mv[((i * 8) + j)] == 1) ? '1' : '0';
            //  Serial.println(((i*8)+j));
        }
        tmpVal = readBinaryString(str_tmp);
#if INFO
        Serial.print("Block: ");
        Serial.print(i);
        Serial.print(" STR=");
        Serial.print(str_tmp);
        Serial.print(" Value: ");
        Serial.println(tmpVal);
#endif
        if (tmpVal > maxVal)
        {
            maxVal = tmpVal; // Set new uppper mark
            region = i;      // Which viewport has the most movement.
        }
    }
    Serial.print("Most activity in block region:");
    Serial.println(region);
    return region;
}

/**
 *
 */
void setup()
{
    uint32_t Freq = 0;
    Serial.begin(115200);
    Freq = getCpuFrequencyMhz();
    Serial.print("CPU Freq = ");
    Serial.print(Freq);
    Serial.println(" MHz");
    Freq = getXtalFrequencyMhz();
    Serial.print("XTAL Freq = ");
    Serial.print(Freq);
    Serial.println(" MHz");
    Freq = getApbFrequency();
    Serial.print("APB Freq = ");
    Serial.print(Freq);
    Serial.println(" Hz");
    Serial.println("Begin Setup...");
    for (uint16_t i = 0; i < VIEWPORT_PIXELS; i++)
    {
        motionView[i] = 0; // Setup Pixel Array
    }
    pinMode(FLASH_PIN, OUTPUT);
    //    Serial.println(setup_camera(FRAME_SIZE,    PIXFORMAT_JPEG) ? "OK" : "ERR INIT");
    Serial.println(setup_camera(FRAME_SIZE, PIXFORMAT_GRAYSCALE) ? "OK" : "ERR INIT");

    stepper.connectToPins(MOTOR_IN1_PIN, MOTOR_IN2_PIN, MOTOR_IN3_PIN, MOTOR_IN4_PIN);
    stepper.setSpeedInStepsPerSecond(STEPS_PER_SECOND);
    stepper.setAccelerationInStepsPerSecondPerSecond(ACCELLERATION_STEPS_PER_SECOND);
    Serial.println("End Setup...");
    tasks.startNow();
}

/**
 *
 */
void loop()
{
    if (FLASH_MODE == 2)
        digitalWrite(FLASH_PIN, HIGH);
    if (!capture_still())
    {
#if INFO
        Serial.println("Failed capture");
#endif
        return;
    }

    if (motion_detect())
    {
#if INFO
        Serial.println("Motion detected");
#endif
    }
    update_frame();

    tasks.execute();
}

/**
 * Capture image and do down-sampling
 */
bool capture_still()
{

    if (FLASH_MODE == 1)
        digitalWrite(FLASH_PIN, HIGH);
    camera_fb_t *frame_buffer = esp_camera_fb_get();
    if (FLASH_MODE == 1)
        digitalWrite(FLASH_PIN, LOW);
    if (!frame_buffer)
        return false;
    // set all 0s in current frame
    memcpy(empty_frame, current_frame, sizeof(empty_frame)); // FAST! Memcopy vs iterations so much faster.

    // down-sample image in blocks
    for (uint32_t i = 0; i < WIDTH * HEIGHT; i++)
    {
        const uint16_t x = i % WIDTH;
        const uint16_t y = floor(i / WIDTH);
        const uint8_t block_x = floor(x / BLOCK_SIZE);
        const uint8_t block_y = floor(y / BLOCK_SIZE);
        const uint8_t pixel = frame_buffer->buf[i];
        const uint16_t current = current_frame[block_y][block_x];
        // average pixels in block (accumulate)
        current_frame[block_y][block_x] += pixel;
    }

    // average pixels in block (rescale)
    for (int y = 0; y < H; y++)
        for (int x = 0; x < W; x++)
            current_frame[y][x] /= BLOCK_SIZE * BLOCK_SIZE;

#if DEBUG
    Serial.println("Current frame:");
    print_frame(current_frame);
    Serial.println("---------------");
#endif

    esp_camera_fb_return(frame_buffer); // Needed to free up camera memory

    return true;
}

/**
 * Compute the number of different blocks
 * If there are enough, then motion happened
 */
bool motion_detect()
{
    uint16_t changes = 0;
    int lastBlock = 0;
    const uint16_t blocks = (WIDTH * HEIGHT) / (BLOCK_SIZE * BLOCK_SIZE);
    for (uint16_t y = 0; y < H; y++)
    {
        for (uint16_t x = 0; x < W; x++)
        {
            float current = current_frame[y][x];
            float prev = prev_frame[y][x];
            float delta = abs(current - prev) / prev;

            // Fill only those areas that meet the threashold.
            if (delta >= BLOCK_DIFF_THRESHOLD)
            {
#if DEBUG
                Serial.println(delta);
                Serial.print("diff\t");
                Serial.print(y);
                Serial.print('\t');
                Serial.println(x);
#endif

                motionView[x] = 1;
                changes++;
            }
        }
    }
    if (changes == 0)
        return false; // don't need to go any further

    // Change screen data into linear (left to right) expression of data.

    // targetPos = ((10 - viewPortToRegion(motionView)) * 10) * STEPS_PER_DEGREE;
    targetPos = viewPortToRegion(motionView) * (FOV_TOTAL_STEPS / 10);

// Display updates for informational purposes.
#if INFO
    Serial.print(":::");
#endif
    // Clear viewport to zero for next detection phase
    for (uint16_t i = 0; i < VIEWPORT_PIXELS; i++)
    {
#if INFO
        Serial.print(motionView[i]);
#endif
        motionView[i] = 0;
    }
#if INFO
    Serial.println(":::");
    Serial.print("Changed ");
    Serial.print(changes);
    Serial.print(" out of ");
    Serial.println(blocks);

    Serial.print("targetPos:");
    Serial.println(targetPos);
#endif
    return (1.0 * changes / blocks) > IMAGE_DIFF_THRESHOLD;
}

/**
 * Copy current frame to previous
 */
void update_frame()
{
    memcpy(prev_frame, current_frame, sizeof(prev_frame)); // FAST! Memcopy vs iterations so much faster.
}

/**
 * For serial debugging
 * @param frame
 */
void print_frame(uint16_t frame[H][W])
{
    for (int y = 0; y < H; y++)
    {
        for (int x = 0; x < W; x++)
        {
            Serial.print(frame[y][x]);
            Serial.print('\t');
        }

        Serial.println();
    }
}

/**
 *  Camera configuration values
 */
bool setup_camera(framesize_t frameSize, pixformat_t PIXEL_FORMAT)
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
    config.pixel_format = PIXEL_FORMAT;
    config.frame_size = frameSize;
    config.jpeg_quality = 12;
    config.fb_count = 1;

    esp_camera_deinit();
    bool ok = esp_camera_init(&config) == ESP_OK;

    sensor_t *sensor = esp_camera_sensor_get();
    sensor->set_framesize(sensor, frameSize);
    sensor->set_brightness(sensor, 2);
    sensor->set_contrast(sensor, 2);
    sensor->set_special_effect(sensor, 2);

    return ok;
}
int readBinaryString(char *s)
{
    int result = 0;
    while (*s)
    {
        result <<= 1;
        if (*s++ == '1')
            result |= 1;
    }
    return result;
}

void captureSnapshotToSD()
{
    Serial.println(setup_camera(FRAME_SIZE, PIXFORMAT_JPEG) ? "OK" : "ERR INIT");
    esp_camera_fb_get();
    Serial.println(setup_camera(FRAME_SIZE, PIXFORMAT_GRAYSCALE) ? "OK" : "ERR INIT");
}
