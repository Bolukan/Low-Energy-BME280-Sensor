# Low Energy Sensor Logger - to MQTT broker

This project combines a bare ESP8266, a LiFePo4 battery and a BME280 sensor. It reads sensor data more frequently than connecting to WiFi and transmitting the data. Limiting WiFi connections, which takes 6 seconds and consumes more than 100mA, will save the battery. However the downside is the readings will become available on the server with delay. 

Data is sent by mqtt messages to a Mqtt broker. It also sends data on the battery voltage and can be configured to send a special action message if the voltage passes a specific treshold.

This is a project with my rtcmem library, a wrapper for the ESP8266 RTC memory. The rtc library supports the strategy to temporary store sensor readings in the
rtc memory which survives deepsleep. The rtcmem can work with variable structs, which will depend on which sensor is used.

This project uses a modified BME280 library to identify the used sensor. The modified library exposes the calibration data. The calibration data is used to calculate a crc32 checksum which is used as identifier.

## Discussion

Please help me and contribute this development:
* Measuring energy use
* Suggesting improvements
* Suggesting use cases

## Latest stable release: not yet available

## Pins
ESP8266 ESP12F on a breakout board + BME280
| ESP8266         | BME280        | LiFePo4 |
| --------------- | ------------- | ------- |
| REST - GPIO16   |               |         |
| ADC             |               |         |
| CH_PD (10K VCC) |               |         |
| GPIO16 - REST   |               |         |
| GPIO14          |               |         |
| GPIO12          |               |         |
| GPIO13          |               |         |
| VCC             | VCC           | PLUS    |
|                 |               |         |
| TXD             |               |         |
| RXD             |               |         |
| GPIO5           | SCL           |         | 
| GPIO4           | SDA           |         |
| GPIO0           |               |         |
| GPIO2           |               |         |
| GPIO15-10K-GND  |               |         |
| GND             | GND           | MIN     |
