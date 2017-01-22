# ESPway

A segway-like balancing two-wheel robot built on ESP8266. It is controlled over WiFi via a HTML/JS GUI and a WebSocket.

This is a work in progress, but basic functionality now works.

The project takes advantage of the [ESP8266 Arduino core](https://github.com/esp8266/Arduino),
[PlatformIO](http://platformio.org/) and
[some](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
[excellent](https://github.com/me-no-dev/ESPAsyncWebServer)
[libraries](https://github.com/Makuna/NeoPixelBus).

## TODO

Beware! There might be some nasty bugs or erratic behavior. Some things TODO:

* Ensure smooth state transitions (motor control)
* Resolve bug where something in the sketch stalls/freezes

## License
The project is licensed under GPLv3.

