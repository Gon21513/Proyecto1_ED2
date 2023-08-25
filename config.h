/************************ Adafruit IO Config *******************************/

// visit io.adafruit.com if you need to create an account,
// or if you need your Adafruit IO key.
#define IO_USERNAME  "xPower"
#define IO_KEY       "aio_PeMP62ds8VpSGM6sVD3wlTtAP2Ln"

/******************************* WIFI **************************************/

#define WIFI_SSID "FoodDispenser"
#define WIFI_PASS "xPowerr_"

#include "AdafruitIO_WiFi.h"

AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASS);