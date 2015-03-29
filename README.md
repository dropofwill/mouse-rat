# mouse_rat

Code for a gesture base Arduino mouse

## "Schematic"

![Rough Schematic](http://github.com/dropofwill/mouse_rat/mouse_rat-schematic.jpg)

## Pictures

![Top Down](http://github.com/dropofwill/mouse_rat/mouse_rat-top-down.jpg)

![Bottom Down](http://github.com/dropofwill/mouse_rat/mouse_rat-bottom-down.jpg)

![Close Up](http://github.com/dropofwill/mouse_rat/mouse_rat-close-up.jpg)

## Dependencies

### For the LSM303:

[adafruit/Adafruit_LSM303](https://github.com/adafruit/Adafruit_LSM303)

### For the LSM9DS0:

[adafruit/Adafruit_LSM9DS0_Library](https://github.com/adafruit/Adafruit_LSM9DS0_Library)

[adafruit/Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)

### For the MPR121:

[adafruit/Adafruit_MPR121_Library](https://github.com/adafruit/Adafruit_MPR121_Library)

You may need to manually adjust the threshold values for touch based on your glove, we ended up using 14 for touch and 7 for release.
