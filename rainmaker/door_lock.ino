// This example demonstrates the ESP RainMaker with a standard Switch device.
#include "RMaker.h"
#include "WiFi.h"
#include "WiFiProv.h"
#include "AppInsights.h"
#include <ESP32Servo.h>

#define DEFAULT_POWER_MODE true
const char *service_name = "PROV_1234";
const char *pop = "abcd1234";

// GPIO for push button, virtual device and light sensor
static int gpio_0 = 0;
static int gpio_servo = 1;
#define LIGHT_SENSOR_PIN 6
#define INPUT_PIN 17

/* Variable for reading pin status*/
bool switch_state = true; //true: Unlocked, false: Locked
Servo my_servo; 

// The framework provides some standard device types like switch, lightbulb,
// fan, temperaturesensor.
static Switch *my_switch = NULL;
//static Device *my_ldr = NULL;

void sysProvEvent(arduino_event_t *sys_event)
{
    switch (sys_event->event_id) {
    case ARDUINO_EVENT_PROV_START:
        Serial.printf("\nProvisioning Started with name \"%s\" and PoP \"%s\" on BLE\n",
                      service_name, pop);
        printQR(service_name, pop, "ble");
        break;
    case ARDUINO_EVENT_PROV_INIT:
        wifi_prov_mgr_disable_auto_stop(10000);
        break;
    case ARDUINO_EVENT_PROV_CRED_SUCCESS:
        wifi_prov_mgr_stop_provisioning();
        break;
    default:;
    }
}

void write_callback(Device *device, Param *param, const param_val_t val,
                    void *priv_data, write_ctx_t *ctx)
{
  //Create a servo component
  //Code out its functionality (states: unlocked, locked)
  //
    const char *device_name = device->getDeviceName();
    const char *param_name = param->getParamName();

    if (strcmp(param_name, "Power") == 0) {
        Serial.printf("Received value = %s for %s - %s\n",
                      val.val.b ? "true" : "false", device_name, param_name);
        switch_state = val.val.b;
        (switch_state == false) ? lock()
        : unlock();
        param->updateAndReport(val);
    }
}

//initialise counter
int i = 0;

void setup()
{
    Serial.begin(115200);
    pinMode(gpio_0, OUTPUT);
    digitalWrite(gpio_0, LOW);
    pinMode(INPUT_PIN, INPUT);
    my_servo.attach(gpio_servo);
    // pinMode(gpio_servo, OUTPUT);
    Serial.println("Servo Attached");
    lock();
    // digitalWrite(gpio_servo, DEFAULT_POWER_MODE);

    Node my_node;
    my_node = RMaker.initNode("ESP RainMaker Node");

    // Initialize switch device
    my_switch = new Switch("Servo", &gpio_servo);
    if (!my_switch) {
        return;
    }

    // int ldrValue = map(analogRead(LIGHT_SENSOR_PIN),0,4095,0,100);

    // Initialize LDR
    // my_ldr = new Device("Light Sensor", ESP_RMAKER_DEVICE_TEMP_SENSOR, NULL);
    // if (!my_ldr) {
    //     return;
    // }

    //Create custom light sensor
    // my_ldr->addNameParam();
    // my_ldr->assignPrimaryParam(my_ldr->getParamByName(ESP_RMAKER_DEF_POWER_NAME));

    // Param brightness("Brightness", ESP_RMAKER_PARAM_TEMPERATURE, value(ldrValue), PROP_FLAG_READ | PROP_FLAG_TIME_SERIES);
    // my_ldr->addParam(brightness);

    // Standard switch device
    my_switch->addCb(write_callback);

    // Add switch device to the node
    my_node.addDevice(*my_switch);
    // my_node.addDevice(*my_ldr);

    // This is optional
    RMaker.enableOTA(OTA_USING_TOPICS);
    // If you want to enable scheduling, set time zone for your region using
    // setTimeZone(). The list of available values are provided here
    // https://rainmaker.espressif.com/docs/time-service.html
    RMaker.setTimeZone("Asia/Shanghai");
    //  Alternatively, enable the Timezone service and let the phone apps set the
    //  appropriate timezone
    RMaker.enableTZService();

    RMaker.enableSchedule();

    RMaker.enableScenes();
    // Enable ESP Insights. Insteads of using the default http transport, this function will
    // reuse the existing MQTT connection of Rainmaker, thereby saving memory space.
    //initAppInsights();  

    RMaker.enableSystemService(SYSTEM_SERV_FLAGS_ALL, 2, 2, 2);

    RMaker.start();

    WiFi.onEvent(sysProvEvent);
    WiFiProv.beginProvision(WIFI_PROV_SCHEME_BLE, WIFI_PROV_SCHEME_HANDLER_FREE_BTDM,
                            WIFI_PROV_SECURITY_1, pop, service_name);
}



void lock() {
  for (int pos = 0; pos <= 90; pos += 5) { // goes from 0 degrees to 180 degrees

    // in steps of 1 degree

    my_servo.write(pos);              // tell servo to go to position in variable 'pos'

    delay(10);                       // waits 15ms for the servo to reach the position

  }

}

void unlock() {
  for (int pos = 90; pos >= 0; pos -= 5) { // goes from 180 degrees to 0 degrees

    my_servo.write(pos);              // tell servo to go to position in variable 'pos'

    delay(10);                       // waits 15ms for the servo to reach the position

  }
}

void loop()
{
  if (digitalRead(INPUT_PIN) == HIGH) {
    Serial.println("Input detected");
    if (!switch_state) { //true: Unlocked, false: Locked
      switch_state = !switch_state;
      if (my_switch) {
        my_switch->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state);
      }
      (switch_state == false) ? lock() : unlock();  

      delay(5000);

      switch_state = !switch_state;
      if (my_switch) {
        my_switch->updateAndReportParam(ESP_RMAKER_DEF_POWER_NAME, switch_state);
      }
      (switch_state == false) ? lock() : unlock();  
    } 
    //(switch_state == false) ? digitalWrite(gpio_0, HIGH) : digitalWrite(gpio_0, LOW);
  }  
}
