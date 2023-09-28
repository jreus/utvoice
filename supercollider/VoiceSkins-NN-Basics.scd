// ==== Basics of Loading a TorchScript File into NN.ar ====

// Specify path to torchscript checkpoint
~path_to_model = "C:/Users/username/models/rave/rave3-musilonfront-44k-1319k-F99.ts";

// Or load the file using a file dialog
Dialog.openPanel({|path| ~path_to_model=path });

postln(~path_to_model);

ServerOptions.devices.do({|dev,idx| postln("%:%".format(idx,dev)) });

(
s.options.device = ServerOptions.devices[15];
s.options.memSize = 2**16;
s.options.hardwareBufferSize = 2048;
//s.options.blockSize = 512;
s.options.blockSize = 2048;
s.boot;
);

NN.load(\musilon, ~path_to_model);
//NN.load(\vctk, ~path_to_model);
NN(\musilon).warmup; // not 100% necessary
NN.describeAll; // print info on all loaded models


if(x.notNil) { x.free }; // if x exists, free it before creating a new synth

(
x={|pan=0.0, gain=1.0|
	var input, output, z;
	var biases, scales;
	biases = 4.do {};
	scales = 4.do {};
	input = SoundIn.ar(0); // audio input from channel 0 on your interface
	//input = PinkNoise.ar() * MouseY.kr(0.1,1.0);
	z = NN(\musilon, \encode).ar(inputs: input);
	z = (z * 1.0) + 0.0; // bias & scaling
	output = NN(\musilon, \decode).ar(inputs: z);
	Pan2.ar(output, pan, gain);
}.play();

);


( // This block makes a simple gui window with four faders and displays it.
w=Window("GUI", Rect(0,0, 250, 300));
~fader1 = Slider(w, Rect(10,0,50,300));
~fader2 = Slider(w, Rect(70,0,50,300));
~fader3 = Slider(w, Rect(130,0,50,300));
~fader4 = Slider(w, Rect(190,0,50,300));
w.front;
);

// Notice the global environment variables ~fader1 ~fader2 ~fader3 ~fader4
// You can define an environment variable with any name starting with ~ and it will be accessible everywhere.

// The faders don't actually do anything yet. To get them to do something when you move them you need to
// set a function (called an "action") to each one like this..
(
~fader1.action = {|sl| postln("Fader1: " ++ sl.value) };
~fader2.action = {|sl| postln("Fader2: " ++ sl.value) };
~fader3.action = {|sl| postln("Fader3: " ++ sl.value) };
~fader4.action = {|sl| postln("Fader4: " ++ sl.value) };
);

// Now if you want to get them controlling the parameters on a synth, you can do this.
(
x = {|f1=261,f2=587,lfo1=8,lfo2=3.7,rq=0.3| RLPF.ar(Saw.ar([f1,f2]) * Saw.ar(lfo1), LFTri.ar(lfo2).range(40, 2200), rq) }.play();
~fader1.action = {|sl| x.set(\f1, sl.value.linlin(0.0, 1.0, 100, 1600)) };
~fader2.action = {|sl| x.set(\f2, sl.value.linlin(0.0, 1.0, 100, 1600)) };
~fader3.action = {|sl| x.set(\lfo1, sl.value.linlin(0.0, 1.0, 0.3, 16)) };
~fader4.action = {|sl| x.set(\lfo2, sl.value.linlin(0.0, 1.0, 0.3, 16)) };
);
// linlin is a method that maps a number from one range to another linearly

x.free; // don't forget to free your synth when you're done with it


