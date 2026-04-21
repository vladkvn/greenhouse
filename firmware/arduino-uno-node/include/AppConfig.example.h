#ifndef GREENHOUSE_APP_CONFIG_EXAMPLE_H
#define GREENHOUSE_APP_CONFIG_EXAMPLE_H

// Template only: copy to AppConfig.h (tracked in git as the default) or merge changes manually.

#define APP_SERIAL_BAUD 115200

#define APP_MQTT_BROKER_IP_1 192
#define APP_MQTT_BROKER_IP_2 168
#define APP_MQTT_BROKER_IP_3 1
#define APP_MQTT_BROKER_IP_4 50

#define APP_MQTT_PORT 1883

#define APP_NODE_ID "uno-test-1"

static const uint8_t APP_ETH_MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

#define APP_TELEMETRY_INTERVAL_MS 15000UL

#define APP_WATER_SENSOR_ANALOG_PIN A0

#define APP_LCD_I2C_ADDR 0x27

#endif  // GREENHOUSE_APP_CONFIG_EXAMPLE_H
