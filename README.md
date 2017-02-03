# ESPway

A segway-like balancing two-wheel robot built on ESP8266. It is controlled over WiFi via a HTML/JS GUI and a WebSocket.

This is a work in progress, but basic functionality now works.

The project takes advantage of the [ESP8266 Arduino core](https://github.com/esp8266/Arduino),
[PlatformIO](http://platformio.org/) and
[some](https://github.com/jrowberg/i2cdevlib/tree/master/Arduino/MPU6050)
[excellent](https://github.com/me-no-dev/ESPAsyncWebServer)
[libraries](https://github.com/Makuna/NeoPixelBus). Instead of Arduino's `analogWrite`,
StephanBruens's [ESP8266_new_pwm](https://github.com/StefanBruens/ESP8266_new_pwm) is used for running the motors.

## TODO

Beware! There might be some nasty bugs or erratic behavior, although some major issues in the early code (watchdog resets, incorrect PID initialization) have been resolved.
Some things TODO:

* Gyro calibration
* PID tuning graphically via the browser

## License
The project is licensed under GPLv3.

