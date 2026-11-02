# Arduino car

Sketch for the Bluetooth-controlled Arduino car (HC-05/HC-06 on pins 0 and 1), driven from Mayday or the IR remote.

## Upload
1. Open `arduino_car/arduino_car.ino` in Arduino IDE (Board: Arduino Uno, or your board; library: Servo, built in).
2. Unplug the Bluetooth module's TX and RX wires (pins 0 and 1) while uploading.
3. Upload, then plug the wires back in.

`arduino_car/test/` holds a host-side test of the command parser (not part of the sketch build).
