#include <Arduino.h>
#include "src/devices_init.h"
#include "src/adpd/u_adpd6100.h"
#include "src/as7341/spec_meas.h"
#include "src/mlx90632/u_mlx.h"
#include "serial.h"
#include "do_command.h"
#include "config.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include <Preferences.h>
#include "Esp.h"
#include "nvs1.h"
#include "openjii_proto.h"

void frontend_json_register();   // src/frontend_json.cpp

static const char* TAG = "INO";
ADPD6 adpd;
Preferences preferences;
uint8_t CONNECTION_TYPE = 0;
bool FLAG_DEICE = false;
static uint16_t sleep_threshod_ms = 100;

static struct Reset_Button{
   unsigned int previous_toggle_t = millis();
   unsigned int counter = 0;
}RB;

void ARDUINO_ISR_ATTR RB_toggle(){
    unsigned int t = millis();
    unsigned int dt = t - RB.previous_toggle_t;
    RB.previous_toggle_t = t;

    if ((dt > 8) && (dt < 12)){
        RB.counter += 1;
    }else{
        RB.counter = 0;
    }
    if (RB.counter > 3){
        RB.counter = 0;
        esp_restart();
        return;
    }
    return;
}


/* Pause / resume the BOOT-pin reset gesture around a measurement.
 * V1 bench (plans/DETERMINISTIC_ADPD.md §8): every triggered run at 100 Hz (and 90 Hz)
 * reset the ESP within ~250 ms with nothing printed — RB_toggle's esp_restart(). GPIO9 is
 * a bare BOOT input on the 45 k internal pull-up, and the LED-driver current edges of a
 * 10 ms-period sequence land exactly in the ISR's 8-12 ms toggle window, so an awake
 * core counts them as button toggles. Free-run mostly sleeps through the pulses, the
 * triggered engine is awake for every one. Same treatment as ambit_light_sleep(): drop
 * the interrupt while it cannot be trusted, re-arm with a fresh count afterwards. */
void ambit_boot_gesture_pause(){
    detachInterrupt(BOOT_PIN);
}
void ambit_boot_gesture_resume(){
    RB.counter = 0;
    RB.previous_toggle_t = millis();
    attachInterrupt(BOOT_PIN, RB_toggle, CHANGE);
}

void ambit_light_sleep(){
    detachInterrupt(BOOT_PIN);
    gpio_sleep_set_direction(GPIO_NUM_9, GPIO_MODE_INPUT);
    gpio_sleep_set_pull_mode(GPIO_NUM_9, GPIO_PULLUP_ONLY);
    gpio_wakeup_enable(GPIO_NUM_9, GPIO_INTR_LOW_LEVEL);
    esp_sleep_enable_gpio_wakeup();
    esp_light_sleep_start();
    attachInterrupt(BOOT_PIN, RB_toggle, CHANGE);
}



 
int serial_read_until(uint8_t target1, uint8_t target2 = 0, uint8_t target3 = 0, uint16_t timeout = 20, bool remove = false);
uint16_t flush_serial(uint8_t timeout);
void setup(){
    esp_timer_early_init();
    pinMode(STF_FLASH_PIN, OUTPUT);
    pinMode(BOOT_PIN, INPUT_PULLUP);
    digitalWrite(STF_FLASH_PIN, LOW);
    pinMode(10, OUTPUT);
    digitalWrite(10, LOW);
    attachInterrupt(BOOT_PIN, RB_toggle, CHANGE);


    Serial.begin(115200);
    delay(250);
    Serial.println("BOOT");
    Serial.println(esp_timer_get_time());
    Serial.setTimeout(50);

    /* The binary FSM protocol shares UART0 with the ESP-IDF log console
     * (Serial -> UART0). Silence all logging so it can't interleave ASCII into
     * the binary stream sent to the ambyte over the FFC. */
    esp_log_level_set("*", ESP_LOG_NONE);


    digitalWrite(STF_FLASH_PIN, HIGH);
    delayMicroseconds(1);
    digitalWrite(STF_FLASH_PIN, LOW);
    init_i2c_bus();
    init_spi_bus();
    adpd.begin();
    if (as7341.begin()) ESP_LOGV(TAG, "AS7341 Found");
    check_AS7341();
    AS_LED_OFF();
    mlx_init();
    CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;    

    uart_set_wakeup_threshold(UART_NUM_0, 3);
    esp_sleep_enable_uart_wakeup(UART_NUM_0);

    gpio_sleep_set_direction(GPIO_NUM_20, GPIO_MODE_INPUT);
    gpio_sleep_set_pull_mode(GPIO_NUM_20, GPIO_PULLUP_ONLY);

    gpio_sleep_set_direction(GPIO_NUM_1, GPIO_MODE_OUTPUT);
    gpio_sleep_set_pull_mode(GPIO_NUM_1, GPIO_PULLDOWN_ONLY);

    load_info_from_nvs(true);

    frontend_json_register();

    esp_sleep_enable_timer_wakeup(10000000);
    

    FLAG_DEICE = false;
    //esp_sleep_enable_timer_wakeup(200000);
}


int do_esp_cmd();
int c = -1;
char choose[50];

/* Sticky host latch: UNKNOWN is the app-discovery default, so it stays awake
 * and quiet until traffic identifies a host. Only the ambyte's 0xAA wake
 * preamble positively latches BINARY and opts into the frozen light-sleep /
 * 0x85 idle-heartbeat contract; an arbitrary high byte must not do that.
 * TEXT also stays awake because the openJII app / Calibratron / a JSON host do
 * not retry and UART sleep-wake (threshold 3 edges) eats in-flight bytes. Its
 * persistent last-activity timestamp releases the latch after a long idle so
 * a device can later be installed on an ambyte without a reboot. */
enum HostLatch : uint8_t { HOST_UNKNOWN = 0, HOST_BINARY, HOST_TEXT };
static HostLatch host_latch = HOST_UNKNOWN;
static uint32_t host_last_activity_ms = 0;
static const uint32_t TEXT_LATCH_RELEASE_MS = 120000;

void loop(){
    // An in-flight JSON envelope owns the stream until it completes or times
    // out; poll() is what fires its 1 s idle timeout, so keep calling it.
    if (ojii::busy()){
        if (Serial.available() > 0) host_last_activity_ms = millis();
        ojii::poll(Serial);
        if (ojii::busy()){ delay(1); return; }
        host_last_activity_ms = millis();
    }

    c = -1;
    int b;

    for (;;) {
        c = Serial.available();
        if (c > 0){
            host_last_activity_ms = millis();
            c = Serial.peek();
            if (c == 255) Serial.read();
            if (c < 255) break;
        }else{
            if (host_latch == HOST_TEXT){
                if (millis() - host_last_activity_ms > TEXT_LATCH_RELEASE_MS){
                    host_latch = HOST_UNKNOWN;
                }
                delay(10);
            }else if (host_latch == HOST_BINARY &&
                      millis() - host_last_activity_ms > sleep_threshod_ms){


                Serial.flush();
                flush_serial(20);
                ambit_light_sleep();
                c = esp_sleep_get_wakeup_cause();
                host_last_activity_ms = millis();
                if (c == 8){
                    sleep_threshod_ms = 1000;
                }else{
                    sleep_threshod_ms = 200;
                }

                Serial.write(AMBIT_BOOT_IDLE);
                Serial.flush();


                //sleep_timer = millis();


            }else{
                delay(10);
                //sleep_threshod_ms = 200;
            }
        }

    }

    /* First-byte router. Every frozen binary framing byte (0x80 0x85 0xA0 0xA1
     * 0xAA 0xB4 0xD2-0xD4 0xDE 0xF0) is >127; both text dialects are printable
     * ASCII, and only the JSON envelope starts with '{' or '['. ojii::poll()
     * must never see binary traffic (it consumes bytes and silently drops >127
     * junk while unlocked), hence peek-then-route. */
    c = Serial.peek();
    if (c > 127) { // binary FSM to the ambyte (frozen wire)
        ojii::reset();   // a stale partial line must not prepend to later text
        while (Serial.available() > 0){

            b = serial_read_until(170, 160, 222, 50, false);
            if (b == 1){ // wake up signal
                host_latch = HOST_BINARY;
                flush_serial(5);
                Serial.write(128);
                sleep_threshod_ms = 500;
            }else if(b == 2){// command
                do_esp_cmd();
                break;
            }else if (b == 3){ // data send reset signal
                ESP_LOGE(TAG, "OUT of sync");
                Serial.read();
                Serial.write(128);
            }else{
                ESP_LOGE(TAG, "Unknown cmd %d", c);
                Serial.read();
                Serial.write(128);
                break;
            }
        }
        host_last_activity_ms = millis();

    }else if (c == '{' || c == '['){ // openJII JSON envelope
        host_latch = HOST_TEXT;
        ojii::poll(Serial);
        host_last_activity_ms = millis();
    }else{
        // Text console: what the openJII app driver and the Calibratron speak.
        // The old `sleep_threshod_ms = 30000` stay-awake heuristic is
        // superseded by the TEXT latch.
        host_latch = HOST_TEXT;
        /* Claim the sink for this host before dispatching. do_esp_cmd() latches
         * CONNECTION_TYPE=AMBYTE on every binary command and nothing used to
         * clear it, so a text command that does not set its own sink (plain
         * `arrun`; arrun1/arrun2/q/r/w all do) would run with the ambyte sink
         * still selected and emit FSM wake traffic at a text host. Verbs that
         * set their own sink still override this. */
        CONNECTION_TYPE = CONNECTION_TYPES::COMPUTER;
        Serial_Input_Chars(choose, ":,", 200, sizeof(choose) - 1);
        do_command(choose);
        host_last_activity_ms = millis();

    }


}
