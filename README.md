# ttn-coverage-mapping

This program is compatible with TTGO ESP32 T-Beam module that has gps, lora and more.

This is used to get gps location and send it to TTN. You can have a ttn-mapper integration
to futher push data from your application for drawing ttn coverage maps.

TTN Mapper is a mapping page to indicate The Things Network LoRaWan gateway coverage at SF7.

## TTN Mapper integration
A nice trick is to use different ports for degugging and production. I use port 1 (default) for debugging and limit ttn-mapper integration only on port 100.

Data is sent as a binary packet to keep it small and must be decoded with ttn payload decoder. Sample decoding scipt included in tt-decoder directory.


## Caviats
GPS satelitte count seems sometimes out of sync/wrong. Currently it is hardcoded to send count 0 which is ignored at decoder sctipt.

## Libraries
Arduino-LMIC (for lorawan) https://github.com/mcci-catena/arduino-lmic , NeoGPS (for gps) https://github.com/SlashDevin/NeoGPS


## Licensing
This is provided under MIT licence.
Note that libraries have there own licenses.


