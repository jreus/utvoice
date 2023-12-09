/* --------------------------------------------------------

Reads incoming OSC / sends OSC commands to Serial2OSC.py connected to an ESPNow coordinator.

----------------------------------------------------------*/


// Incoming OSC
OSCdef(\esp32, {|msg| msg.postln }, "/esp32");

~esp = NetAddr("localhost", 1337); // addr of Serial2OSC.py
~esp.sendMsg(\fromsc, 0, "s0", 120); // send a message to node 0, servo 0 set to 120 degrees.

(
20.do { ~esp.sendMsg(\fromsc, 0, "s0", rand(180)) };
)