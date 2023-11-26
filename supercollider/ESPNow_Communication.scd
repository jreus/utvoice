// Basic Serial Reciever coming from an ESP32 connected as an AP node

/* --------------------------------------------------------

Starts the serial2osc utility and sets up serial data OSCdefs

----------------------------------------------------------*/

OSCdef(\esp32, {|msg| msg.postln }, "/esp32");
