
// This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License
// as published by the Free Software Foundation, 
// either version 2 of the License, or (at your option) any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
// without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. 
//See the GNU General Public License for more details.

// You should have received a copy of the GNU General Public License along with this program. 
// If not, see <https://www.gnu.org/licenses/>.

// This program is based the ESP32 Frequency Meter 
// by Rui Viana and Gustavo Murta august/2020
// https://blog.eletrogate.com/esp32-frequencimetro-de-precisao

// The hardware used is ESP32-S WiFi Bluetooth Development Board 0,96 OLED Display WROOM 32 NodeMCU 

#include "stdio.h"                                                        // Library STDIO
#include "driver/ledc.h"                                                  // Library ESP32 LEDC
#include "driver/pcnt.h"                                                  // Library ESP32 PCNT
#include "SSD1306.h"                                                      // from Esp8266 and Esp32 OLED driver .. ThingPulse, Fabrice Weinberg
                                                                          // https://github.com/ThingPulse/esp8266-oled-ssd1306
SSD1306  display(0x3c, 5, 4);

#define PCNT_COUNT_UNIT       PCNT_UNIT_0                                 // Set Pulse Counter Unit - 0 
#define PCNT_COUNT_CHANNEL    PCNT_CHANNEL_0                              // Set Pulse Counter channel - 0 

#define PCNT_INPUT_SIG_IO     GPIO_NUM_36                                 // Set Pulse Counter input - Freq Meter Input GPIO 34
#define LEDC_HS_CH0_GPIO      GPIO_NUM_16                                 // Saida do LEDC - gerador de pulsos - GPIO_33
#define PCNT_INPUT_CTRL_IO    GPIO_NUM_25                                 // Set Pulse Counter Control GPIO pin - HIGH = count up, LOW = count down  
#define OUTPUT_CONTROL_GPIO   GPIO_NUM_26                                 // Timer output control port - GPIO_32
#define PCNT_H_LIM_VAL        overflow                                    // Overflow of Pulse Counter 

#define IN_BOARD_LED          GPIO_NUM_2                                  // ESP32 native LED - GPIO 2

bool            flag          = true;                                     // Flag to enable print frequency reading
uint32_t        overflow      = 20000;                                    // Max Pulse Counter value
int16_t         pulses        = 0;                                        // Pulse Counter value
uint32_t        multPulses    = 0;                                        // Quantidade de overflows do contador PCNT
uint32_t        sample_time   = 100000;                                   // 1ooms |  sample time of 1 second to count pulses 1000000
uint32_t        osc_freq      = 1254312;                                  // Oscillator frequency - initial 12543 Hz (may be 1 Hz to 40 MHz)
uint32_t        mDuty         = 0;                                        // Duty value
uint32_t        resolution    = 0;                                        // Resolution value
float           frequency     = 0;                                        // frequency value
char            buf[32];                                                  // Buffer

uint32_t        factor        = 1;                                        // factor in sample_time is different jfs
uint32_t        frequenz      = 0;                                        // resulting frequency jfs
uint32_t        save_osc      = 0;                                        // for output jfs

esp_timer_create_args_t create_args;                                      // Create an esp_timer instance
esp_timer_handle_t timer_handle;                                          // Create an single timer

portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;                     // portMUX_TYPE to do synchronism
int retry = 0;

void setup() {
  Serial.begin(115200);                                                   // Serial Console Arduino 115200 Bps
  Serial.println(" Input the Frequency - 1 to 40 MHz");                   // Console print
  display.init();
  display.flipScreenVertically();
  display.clear();
  display.drawString(0, 0, "Starting...");
  display.display();
  init_frequencyMeter ();                                                 // Initialize Frequency Meter
}
//----------------------------------------------------------------------------
void init_osc_freq ()                                                     // Initialize Oscillator to test Freq Meter
{
  save_osc = osc_freq;                                                    // jfs for output
  resolution = (log (80000000 / osc_freq)  / log(2)) / 2 ;                // Calc of resolution of Oscillator
  if (resolution < 1) resolution = 1;                                     // set min resolution 
  // Serial.println(resolution);                                          // Print
  mDuty = (pow(2, resolution)) / 2;                                       // Calc of Duty Cycle 50% of the pulse
  // Serial.println(mDuty);                                               // Print

  ledc_timer_config_t ledc_timer = {};                                    // LEDC timer config instance

  ledc_timer.duty_resolution =  ledc_timer_bit_t(resolution);             // Set resolution
  ledc_timer.freq_hz    = osc_freq;                                       // Set Oscillator frequency
  ledc_timer.speed_mode = LEDC_HIGH_SPEED_MODE;                           // Set high speed mode
  ledc_timer.timer_num = LEDC_TIMER_0;                                    // Set LEDC timer index - 0
  ledc_timer_config(&ledc_timer);                                         // Set LEDC Timer config

  ledc_channel_config_t ledc_channel = {};                                // LEDC Channel config instance

  ledc_channel.channel    = LEDC_CHANNEL_0;                               // Set HS Channel - 0
  ledc_channel.duty       = mDuty;                                        // Set Duty Cycle 50%
  ledc_channel.gpio_num   = LEDC_HS_CH0_GPIO;                             // LEDC Oscillator output GPIO 33
  ledc_channel.intr_type  = LEDC_INTR_DISABLE;                            // LEDC Fade interrupt disable
  ledc_channel.speed_mode = LEDC_HIGH_SPEED_MODE;                         // Set LEDC high speed mode
  ledc_channel.timer_sel  = LEDC_TIMER_0;                                 // Set timer source of channel - 0
  ledc_channel_config(&ledc_channel);                                     // Config LEDC channel
}

//----------------------------------------------------------------------------------
static void IRAM_ATTR pcnt_intr_handler(void *arg)                        // Counting overflow pulses
{
  portENTER_CRITICAL_ISR(&timerMux);                                      // disabling the interrupts
  multPulses++;                                                           // increment Overflow counter
  PCNT.int_clr.val = BIT(PCNT_COUNT_UNIT);                                // Clear Pulse Counter interrupt bit
  portEXIT_CRITICAL_ISR(&timerMux);                                       // enabling the interrupts
}

//----------------------------------------------------------------------------------
void init_PCNT(void)                                                      // Initialize and run PCNT unit
{
  pcnt_config_t pcnt_config = { };                                        // PCNT unit instance

  pcnt_config.pulse_gpio_num = PCNT_INPUT_SIG_IO;                         // Pulse input GPIO 34 - Freq Meter Input
  pcnt_config.ctrl_gpio_num = PCNT_INPUT_CTRL_IO;                         // Control signal input GPIO 35
  pcnt_config.unit = PCNT_COUNT_UNIT;                                     // Unidade de contagem PCNT - 0
  pcnt_config.channel = PCNT_COUNT_CHANNEL;                               // PCNT unit number - 0
  pcnt_config.counter_h_lim = PCNT_H_LIM_VAL;                             // Maximum counter value - 20000
  pcnt_config.pos_mode = PCNT_COUNT_INC;                                  // PCNT positive edge count mode - inc
  pcnt_config.neg_mode = PCNT_COUNT_INC;                                  // PCNT negative edge count mode - inc
  pcnt_config.lctrl_mode = PCNT_MODE_DISABLE;                             // PCNT low control mode - disable
  pcnt_config.hctrl_mode = PCNT_MODE_KEEP;                                // PCNT high control mode - won't change counter mode
  pcnt_unit_config(&pcnt_config);                                         // Initialize PCNT unit

  pcnt_counter_pause(PCNT_COUNT_UNIT);                                    // Pause PCNT unit
  pcnt_counter_clear(PCNT_COUNT_UNIT);                                    // Clear PCNT unit

  pcnt_event_enable(PCNT_COUNT_UNIT, PCNT_EVT_H_LIM);                     // Enable event to watch - max count
  pcnt_isr_register(pcnt_intr_handler, NULL, 0, NULL);                    // Setup Register ISR handler
  pcnt_intr_enable(PCNT_COUNT_UNIT);                                      // Enable interrupts for PCNT unit

  pcnt_counter_resume(PCNT_COUNT_UNIT);                                   // Resume PCNT unit - starts count
}

//----------------------------------------------------------------------------------
void read_PCNT(void *p)                                                   // Read Pulse Counter
{
  gpio_set_level(OUTPUT_CONTROL_GPIO, 0);                                 // Stop counter - output control LOW
  pcnt_get_counter_value(PCNT_COUNT_UNIT, &pulses);                       // Read Pulse Counter value
  flag = true;                                                            // Change flag to enable print
}

//---------------------------------------------------------------------------------
void init_frequencyMeter ()
{
  init_osc_freq();                                                        // Initialize Oscillator
  init_PCNT();                                                            // Initialize and run PCNT unit

  gpio_pad_select_gpio(OUTPUT_CONTROL_GPIO);                              // Set GPIO pad
  gpio_set_direction(OUTPUT_CONTROL_GPIO, GPIO_MODE_OUTPUT);              // Set GPIO 32 as output

  create_args.callback = read_PCNT;                                       // Set esp-timer argument
  esp_timer_create(&create_args, &timer_handle);                          // Create esp-timer instance

  gpio_set_direction(IN_BOARD_LED, GPIO_MODE_OUTPUT);                     // Set LED inboard as output

  gpio_matrix_in(PCNT_INPUT_SIG_IO, SIG_IN_FUNC226_IDX, false);           // Set GPIO matrin IN - Freq Meter input
  gpio_matrix_out(IN_BOARD_LED, SIG_IN_FUNC226_IDX, false, false);        // Set GPIO matrix OUT - to inboard LED
}

//----------------------------------------------------------------------------------------
char *ultos_recursive(unsigned long val, char *s, unsigned radix, int pos) // Format an unsigned long (32 bits) into a string
{
  int c;
  if (val >= radix)
    s = ultos_recursive(val / radix, s, radix, pos + 1);
  c = val % radix;
  c += (c < 10 ? '0' : 'a' - 10);
  *s++ = c;
  if (pos % 3 == 0) *s++ = ',';
  return s;
}
//----------------------------------------------------------------------------------------
char *ltos(long val, char *s, int radix)                                  // Format an long (32 bits) into a string
{
  if (radix < 2 || radix > 36) {
    s[0] = 0;
  } else {
    char *p = s;
    if (radix == 10 && val < 0) {
      val = -val;
      *p++ = '-';
    }
    p = ultos_recursive(val, p, radix, 0) - 1;
    *p = 0;
  }
  return s;
}

uint32_t start = 100000;

void test(){
  if (start < 100){
    start = 100000;
  } else {
    osc_freq = start;
    init_osc_freq ();
    start -= 100;
  }
}


void loop() {
 if (flag == true)                                                      // If count has ended
  {
    flag = false;                                                       // change flag to disable print
    frequency = (pulses + (multPulses * overflow)) / 2  ;               // Calculation of frequency
    factor = 1000000/sample_time;
    frequenz = frequency*factor;   
    dtostrf(frequency,0,0,buf);
    String s = String(buf);
    s = ">"+s+"<";
    Serial.println(s);

    display.clear();  
    display.drawString(0, 10,"sample time ms ");
    display.drawString(80,10,String(sample_time/1000));
    display.drawString(0, 20,"Pulses SVP");
    display.drawString(60, 20, (ltos(frequency, buf, 10)));
    display.drawString(0, 30,"Freq");
    display.drawString(60, 30, (ltos(frequenz, buf, 10)));
    display.drawString(0, 40,"Oscila 16");
    display.drawString(60, 40, (ltos(save_osc, buf, 10)));
    display.display();

    multPulses = 0;                                                     // Clear overflow counter
   
    delay (5);                                                          // Delay 5 ms 

    pcnt_counter_clear(PCNT_COUNT_UNIT);                                // Clear Pulse Counter
    esp_timer_start_once(timer_handle, sample_time);                    // Initialize High resolution timer (1 sec)
    gpio_set_level(OUTPUT_CONTROL_GPIO, 1);                             // Set enable PCNT count
  }

  String inputString = "";                                               // clear temporary string
  osc_freq = 0;                                                          // Clear oscillator frequency
  while (Serial.available())
  {
    char inChar = (char)Serial.read();                                   // Reads a byte on the console
    inputString += inChar;                                               // Add char to string
    if (inChar == '\n')                                                  // If new line (enter)
    {
      Serial.println(inputString);

      if (inputString.startsWith("s")){                                 // set sample time ms 
        inputString.remove(0,1);
        Serial.println(inputString);
        sample_time = inputString.toInt();
        Serial.println(sample_time);
      }
      if (inputString.startsWith("o")){                                 // set frequency
        inputString.remove(0,1);
        Serial.println(inputString);
        osc_freq = inputString.toInt();
        Serial.println(sample_time);
      }

      // Converts String into integer value
      inputString = "";                                                  // Clear string
    }
  }
  if (osc_freq != 0)                                                     // If some value inputted to oscillator frequency
  {
    init_osc_freq (); 
  }       
}
