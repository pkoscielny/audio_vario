# Audio variometer
An electric variometer indicates climbing and sinking in the air by audio tones. 
Additionally this vario is able to send the data (air pressure and temperature) to an external device using LK8EX1 protocol via Bluetooth.
The altitude taken from the GPS system is definitely much less accurate than the altitude calculated on the basis of the current pressure.
You can hear the climbing or sinking rate and see the altitude in the external device.
The external device can be e.g. a smartfon with LK8000 software installed and configured.
If you want to see the proper altitude you should calibrate the current pressure according to the current altitude before flight. 
This can be done by two buttons (increase or decrease the current altitude).

## Hardware

### List of components
* Arduino Nano
* BMP280 pressure and temperature sensor
* 3.3V <---> 5V logic converter
* HC-06 Bluetooth module
* buzzer
* potentiometer (for buzzer) with integrated on-off switch
* 2x push buttons
* 2x 10k resistors (for buttons)
* 2x 4.7k resistors (for I2C)
* 500mA DC-DC boost converter step up 1-5V -> 5V
* 2x Ni-MH 1.2V batteries
 
### Breadboard circuit diagram
![circuit](/fz/audio_vario.png)

### Device
![view](/img/audio_vario_inside.jpg)

![view](/img/audio_vario_outside.jpg)

Please do not rely on this device during actual flight.
Be careful and use only certified devices when making decisions.
