// Basic Serial Reciever coming from an ESP32 connected as an AP node

/* --------------------------------------------------------

Starts the serial2osc utility and sets up serial data OSCdefs

----------------------------------------------------------*/


MIDIClient.init;

OSCdef(\esp32, {|msg| msg.postln }, "/esp32");

// Using a Serial2OSC converter...

(
a = ();
a.serial = ();
a.serial.radioSilenceThresh = 3; // 3 second until radio silence signal is triggered
a.serial.pid = "/root/BelaUtils/serial2osc/serial2osc --port /dev/ttyS5 --baud 115200 --remote localhost:57120".unixCmd({|ec, pid| postln("serial2osc (pid %) exited with code %".format(pid, ec)) }, true);
a.serial.lastMsgTime = Process.elapsedTime;
// override these functions to respond to incoming signal values
a.serial.signalReceived = {|e, signals| // signal strengths are scaled 0.0-1.0
	if(a.verbose.notNil) { signals.postln };
};
a.serial.radioSilence = {|e, lastTime|
	if(a.verbose.notNil) {
		postln("Radio Silence, last message at %".format(lastTime));
	};
};

postln("Listening for incoming OSC data...");
OSCdef(\serial2osc, {|msg, time|
	var signals = msg[1..].lincurve(0, 1023, 0.0, 1.0, -2);
	a.serial.lastMsgTime = time;
	a.serial.signalReceived(signals);
}, "/serial");

// Routine to check for radio silence and respond accordingly
a.serial.r_radioSilence = Task({
	var now;
	inf.do {
		a.serial.radioSilenceThresh.wait;
		now = Process.elapsedTime;
		if((now - a.serial.lastMsgTime) > a.serial.radioSilenceThresh) {
			// Radio Silence Detected, respond accordingly
			a.serial.radioSilence(a.serial.lastMsgTime);
		};
	};
}).play(AppClock);


);