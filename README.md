# ttn-coverage-mapping

This program is compatible with TTGO T-Beam module that has gps, lora and more.
This is used to get gps location and send data to TTN. You can have a ttn-mapper integration
there to futher push data for drawing ttn coverage maps.

## TTN Mapper integration
A nice trick is to use different ports for degugging and production. I use port 1 (default) for debugging and limit ttn-mapper integration only on port 100.

Data is sent as a binary packet and must be decoded with ttn payload decoder. Sample decoding scipt included in tt-decoder directory.


## Caviats
GPS satelitte count seems sometimes out of sync/wrong. Hardcode to send 0 as satellite count if needed.

## Libraries
Arduino-LMIC (for lorawan) https://github.com/SlashDevin/NeoGPS, NeoGPS (for gps) https://github.com/matthijskooijman/arduino-lmic


## Licensing
This is provided under MIT licence.
Note that libraries have there own licenses.


