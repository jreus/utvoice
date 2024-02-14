/* --------------------------------------------------------

Reads incoming OSC / sends OSC commands to Serial2OSC.py connected to an ESPNow coordinator.
Using Lucas' 4th prototype
which has the configuration in Trill DIFF mode of....

_________ 12
_________ 11
_________ 10
_________ 9
_________ 8
_________ 7
_________ 6
_________ 5
_________ 4
_________ 3
|   |   |
0   1   2

----------------------------------------------------------*/


s.boot;
s.meter;

// Basic Incoming OSC
OSCdef(\esp32, {|msg| msg.postln }, "/esp32");


(
Ndef(\audiotest, {
	SinOsc.ar([1,2,3]*400).sum * 0.1;
}).play;
);
Ndef(\audiotest).clear;




(
// Notey Patch

// Control for Notey Patch
OSCdef(\esp32, {|msg|
	var gains = Array.newClear(13);
	msg = msg[2..];
	msg.postln;
	13.do {|idx|
		var param = "v%".format(idx+1).asSymbol;
		gains[idx] = msg[idx].explin(50, 300, 0.0, 1.0, clip: \minmax);
		Ndef(\notey).set(param, gains[idx]);
		//"% %".format(param, gains[idx]).postln;
	};
	gains.postln;
}, "/esp32");

// Synthesis for Notey patch
Ndef(\notey, {|pregain=10.0, gain=1.0|
	//var pitches = ([1,2,3,4,5,6,7,8,9,11,12,13] + 36).midicps;
	var pitches = (Scale.major.ratios ++ (Scale.major.ratios[..5] * 2)) * 200;
	var voicein = SoundIn.ar(0);
	var sig, carriers;
	var volumes = [
		\v1.kr(0.0, 0.1),
		\v2.kr(0.0, 0.1),
		\v3.kr(0.0, 0.1),
		\v4.kr(0.0, 0.1),
		\v5.kr(0.0, 0.1),
		\v6.kr(0.0, 0.1),
		\v7.kr(0.0, 0.1),
		\v8.kr(0.0, 0.1),
		\v9.kr(0.0, 0.1),
		\v10.kr(0.0, 0.1),
		\v11.kr(0.0, 0.1),
		\v12.kr(0.0, 0.1),
		\v13.kr(0.0, 0.1),
		0,0,0,0,0,0,0,
	];
	//volumes = volumes + 1.0;

	carriers = Saw.ar(pitches) * volumes;
	sig = Vocoder.ar(carriers, voicein, 20, 200, 5000, 0.02, 5000, 0.05, 25) * pregain;

	//sig = Splay.ar(sig); // spread voices in the stereo field
	sig = Pan2.ar(sig.sum, 0.0); // mix down to mono and pan how you like
	sig = sig * gain;
	Limiter.ar(sig);

}).play;
);

//Ndef(\notey).set(\v5, 1.0);

( // Voice-free Notey for interaction testing.

Ndef(\notey, {|pregain=10.0, gain=1.0|
	//var pitches = ([1,2,3,4,5,6,7,8,9,11,12,13] + 36).midicps;
	var pitches = (Scale.major.ratios ++ (Scale.major.ratios * 2)) * 200;
	var voicein = SoundIn.ar(0);
	var sig, carriers;
	var volumes = [
		\v1.kr(0.0, 0.1),
		\v2.kr(0.0, 0.1),
		\v3.kr(0.0, 0.1),
		\v4.kr(0.0, 0.1),
		\v5.kr(0.0, 0.1),
		\v6.kr(0.0, 0.1),
		\v7.kr(0.0, 0.1),
		\v8.kr(0.0, 0.1),
		\v9.kr(0.0, 0.1),
		\v10.kr(0.0, 0.1),
		\v11.kr(0.0, 0.1),
		\v12.kr(0.0, 0.1),
		\v13.kr(0.0, 0.1),
		0,0,0,0,0,0,0,0,
	];
	//volumes = volumes + 1.0;

	carriers = SinOsc.ar(pitches) * volumes;
	//sig = Vocoder.ar(carriers, voicein, 20, 200, 5000, 0.02, 5000, 0.05, 25) * pregain;
	sig = carriers;

	//sig = Splay.ar(sig); // spread voices in the stereo field
	sig = Pan2.ar(sig.sum, 0.0); // mix down to mono and pan how you like
	sig = sig * gain;
	Limiter.ar(sig);

}).play;

);


(

~current_mode = 0;
~current_intensity = 0;


// Control for FX Patch
OSCdef(\esp32, {|msg|
	var modevals = [msg[2], msg[3], msg[4]];
	var modestate = -1;
	var intensityslider = msg[4..];
	var intensity = intensityslider.maxIndex;

	"INTENSITY: %   MODE: %".format(intensity, modestate).postln;


	if(modevals.sum < 40) {
		modestate = -1;
	} {
		modestate = modevals.maxIndex;
		~current_mode = modestate;
	};




	//msg = msg[2..];
	//msg.postln;
	// "MODEVALS: %  --- CHOOSE: %".format(modevals, modevals.maxIndex).postln;
	// "INTENSITY: % --- INTENSITIES: %".format(intensityslider.maxIndex, intensityslider).postln;
	// var gains = Array.newClear(13);
	// 13.do {|idx|
	// 	var param = "v%".format(idx+1).asSymbol;
	// 	gains[idx] = msg[idx].explin(50, 300, 0.0, 1.0, clip: \minmax);
	// 	Ndef(\notey).set(param, gains[idx]);
	// 	//"% %".format(param, gains[idx]).postln;
	// };
	// gains.postln;
}, "/esp32");


);


( // FX Patch

Ndef(\notey, {|gain=1.0| // This is an FX patch! We call it Notey to make switching between synths easier, no need to free previous notey!

	//var pitches = ([1,2,3,4,5,6,7,8,9,11,12,13] + 36).midicps;
	var revsig, distsig, delsig, mix;
	var voicein = SoundIn.ar(0);

	distsig = (voicein * \predist.kr(10.0, 0.1)).softclip;
	delsig = CombC.ar(voicein, 1.0, \deltime.kr(0.4, 0.1), \deldecay.kr(0.5, 0.1));

	mix = [voicein * \voicemix.kr(1.0, 0.1), distsig * \distmix.kr(1.0, 0.1), delsig * \delmix.kr(1.0, 0.1)].sum;
	revsig = JPverb.ar(mix, t60: \revtime.kr(3.0, 0.1), damp: 0.3, size: \revsize.kr(3.0, 0.1));

	mix = (mix * \noverbmix.kr(0.2, 0.1)) + (revsig * \revmix.kr(1.0, 0.1));

	Limiter.ar(mix * gain);

}).play;

);







~esp = NetAddr("localhost", 1337); // addr of Serial2OSC.py
~esp.sendMsg(\fromsc, 0, "s0", 120); // send a message to node 0, servo 0 set to 120 degrees.

(
20.do { ~esp.sendMsg(\fromsc, 0, "s0", rand(180)) };
)