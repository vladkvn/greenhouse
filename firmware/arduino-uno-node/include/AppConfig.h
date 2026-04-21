#ifndef GREENHOUSE_APP_CONFIG_H
#define GREENHOUSE_APP_CONFIG_H

// USB serial (default `uno` transport = transport_serial.cpp).
#define APP_SERIAL_BAUD 115200

// Network: used only by `uno-mqtt-eth` (Ethernet + MQTT). Set the broker to the LAN IP of the PC running Mosquitto.
#define APP_MQTT_BROKER_IP_1 192
#define APP_MQTT_BROKER_IP_2 168
#define APP_MQTT_BROKER_IP_3 1
#define APP_MQTT_BROKER_IP_4 50

#define APP_MQTT_PORT 1883

// Must stay stable for topic paths and MQTT client id (fits in PubSubClient id buffer).
#define APP_NODE_ID "uno-test-1"

// Unique on your LAN when using DHCP.
static const uint8_t APP_ETH_MAC[6] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};

#define APP_TELEMETRY_INTERVAL_MS 15000UL

// Analog water level sensor (typical FC-37 style module: AO -> pin).
#define APP_WATER_SENSOR_ANALOG_PIN A0

// PCF8574 backpack address (try 0x3F if the display is blank).
#define APP_LCD_I2C_ADDR 0x27

#endif
