# utvoice
A software framework supporting artistic research on wearable robotics and wearable voice performance interfaces.

Currently this repository contains ESP32 firmware and python scripts for using artistic protocols such as OSC and MIDI to communicate with a mesh network of ESP32 boards including sensor nodes and actuator nodes.

The respository includes:
* `python`          Python scripts for converting incoming serial data from a ESP32 coordinator to protocols usable by music softwares: such as OSC and MIDI.
* `firmware`        ESP32 firmware for different purposes: coordinator, sensor node, actuator node, hardware tests etc..
* `supercollider`   SuperCollider code for communicating with wireless wearable electronics systems built using `utvoice`

