/* --------------------------------------------------------

Reads incoming OSC / sends OSC commands to Serial2OSC.py connected to an ESPNow coordinator.
Using Lucas' 4th prototype which has the touch pad configuration in Trill DIFF mode of....




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

[TRILL CONNECTOR]

Mapping for Notey Synth:
0-12 all correspond to pitch values, keyboard style.. but remember that we don't get discrete keyboard-style control so easily
with a trill configured this way. Touching one sensor pad will always give you some capacitive "spillover" into the surrounding pads.


Mapping for FX Synth:
0, 1, 2 activate control over Reverb, Distortion and Delay FX respectfully
3-12 works as an intensity slider for the currently active FX

----------------------------------------------------------*/

// Boot the server
(
s.options.memSize = 2 ** 16;
s.waitForBoot {
	s.meter
};
);

( // Basic Incoming OSC test.. just print whatever Python is sending us.
OSCdef(\esp32, {|msg| msg.postln }, "/esp32");
);
OSCdef(\esp32).clear;

( // Basic Stereo Audio Test. Some stereo Sine waves
Ndef(\audiotest, {
	Splay.ar(SinOsc.ar(Scale.minor.ratios * 300) * 0.2);
}).play;
);
Ndef(\audiotest).clear; // destroy the synth



/*-----------------------------------------------------------------
██    ██  ██████   ██████  ██████  ██████  ███████ ██████
██    ██ ██    ██ ██      ██    ██ ██   ██ ██      ██   ██
██    ██ ██    ██ ██      ██    ██ ██   ██ █████   ██████
 ██  ██  ██    ██ ██      ██    ██ ██   ██ ██      ██   ██
  ████    ██████   ██████  ██████  ██████  ███████ ██   ██
                                                           */
( // Vocoder OSC Processing
OSCdef(\esp32, {|msg|
	var gains = Array.newClear(13);
	msg = msg[2..15];

	// Map all 13 sensor values to gain levels for each "note"
	13.do {|idx|
		var param = "v%".format(idx+1).asSymbol;
		gains[idx] = msg[idx].explin(50, 300, 0.0, 1.0, clip: \minmax); // map range 50:300 to 0.0:1.0 with exponential curve
		Ndef(\synthpatch).set(param, gains[idx]);
		//"% %".format(param, gains[idx]).postln;
	};
	//msg.join(":").postln;
	//gains.postln;
}, "/esp32");
);
Ndef(\audiotest).clear; // destroy the synth


( // Notey Vocoder Synth
Ndef(\synthpatch, {|pregain=1.0, gain=1.0|
	var pitchratios = ((Scale.major.ratios / 2) ++ Scale.major.ratios)[..12]; // experiment with other distributions of pitch ratios
	//var pitches = (Array.series(13, 1, 3) + 36).midicps;
	var voicein = SoundIn.ar(0);
	var sig, shifted, voxmix;
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
	];

	shifted = PitchShift.ar(LPF.ar(voicein, 4000), 0.2, pitchRatio: pitchratios, pitchDispersion: 0.01, timeDispersion: 0.01);
	shifted = shifted * volumes;

	//sig = Splay.ar(shifted); // spread voices across the stereo field
	//sig = Pan2.ar(Mix(shifted), 0.0); // mix down to mono and pan how you like
	sig = GVerb.ar(Mix(shifted), 50, 2, 0.5, 0.5, 5, 1, 0.5, 0.3); // run into a stereo reverb
	sig = sig * gain;
	Limiter.ar(sig);

}).play;
);

( // Notey Sinewave synth (voice input is ignored)

Ndef(\synthpatch, {|pregain=10.0, gain=1.0|
	//var pitches = ([1,2,3,4,5,6,7,8,9,11,12,13] + 36).midicps;
	var pitches = (Scale.major.ratios ++ (Scale.major.ratios * 2)) * 200;
	var sig;
	var volumes = [
		\v1.kr(0.0, 0.5),
		\v2.kr(0.0, 0.5),
		\v3.kr(0.0, 0.5),
		\v4.kr(0.0, 0.5),
		\v5.kr(0.0, 0.5),
		\v6.kr(0.0, 0.5),
		\v7.kr(0.0, 0.5),
		\v8.kr(0.0, 0.5),
		\v9.kr(0.0, 0.5),
		\v10.kr(0.0, 0.5),
		\v11.kr(0.0, 0.5),
		\v12.kr(0.0, 0.5),
		\v13.kr(0.0, 0.5),
	];
	sig = RLPF.ar(SinOsc.ar(pitches) * 0.3 * volumes, 2000, 0.4);
	sig = sig + CombC.ar(sig, 1.0, 0.01, 2.5);
	sig = Pan2.ar(sig.sum.tanh, 0.0);
	//sig = Splay.ar(sig.tanh, 10, 1.0, 0.0);
	Limiter.ar(sig * gain);
}).play;

);



/*------------------------------------------------------
██████  ███████ ██    ██ ███████ ██████  ██████
██   ██ ██      ██    ██ ██      ██   ██ ██   ██
██████  █████   ██    ██ █████   ██████  ██████
██   ██ ██       ██  ██  ██      ██   ██ ██   ██
██   ██ ███████   ████   ███████ ██   ██ ██████
                                                  */
( // Reverb OSC Processing
~current_slider = 0;
~current_smallslider = 0;
OSCdef(\esp32, {|msg|
	var smallslider = [msg[2], msg[3], msg[4]];
	var intensityslider = msg[4..15];
	var valueschanged = false;
	var smallintensity = smallslider.maxIndex; // 0-2
	var intensity = intensityslider.maxIndex; // will be an intensity index from 0-9


	if(smallslider.sum > 40) { // threshhold
		~current_smallslider = smallintensity;
		valueschanged=true;
	};
	if(intensityslider.sum > 200) { // threshhold below which slider touches are ignored..
		~current_slider = intensity;
		valueschanged=true;
	};

	if(valueschanged) {
		var distlevel = ~current_smallslider.linexp(0.0, 2.0, 0.1, 0.8);
		var drylevel = ~current_smallslider.linlin(0.0, 2.0, 0.7, 0.3);
		var revtime = ~current_slider.linexp(0, 9, 2, 20);
		"Set distlevel: %  Revtime: %".format(distlevel, revtime).postln;
		Ndef(\synthpatch).set(\drylevel, drylevel, \distlevel, distlevel, \revlevel, 1.0, \revtime, revtime, \damp, 0.5);
	};

	"SLIDER: %   SMALL SLIDER: %".format(~current_slider, ~current_smallslider).postln;


}, "/esp32");
);
OSCdef(\esp32).clear;


( // Distorted Reverb Synthesis Patch
Ndef(\synthpatch, {|pregain=1.0, gain=1.0, drylevel=0.1, distlevel=0.9, revlevel=1.0, revsize=50|
	var revsig, distsig, distmix, mix;
	var voicein = SoundIn.ar(0) * pregain;
	distsig = Compander.ar((voicein * 100).softclip, voicein, 0.4, 2/1, 1/100);
	distmix = (voicein * drylevel) + (distsig * distlevel);
	revsig = GVerb.ar(distmix, revsize, revtime: \revtime.kr(1.0, 0.5), damping: \damp.kr(0.7, 0.1), drylevel: 0, earlyreflevel: 0.2, taillevel: 0.3, maxroomsize: 300);
	mix = distmix + (revsig * revlevel);

	Limiter.ar(mix * gain);
}).play;
);
Ndef(\synthpatch).clear;


Ndef(\synthpatch).set(\drylevel, 0.2, \distlevel, 0.5, \revlevel, 0.9, \revtime, 20, \damp, 0.6);



