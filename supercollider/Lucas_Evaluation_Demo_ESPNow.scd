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


s.boot;
s.meter;

( // Basic Incoming OSC test.. just print whatever Python is sending us.
OSCdef(\esp32, {|msg| msg.postln }, "/esp32");
);

( // Basic Audio Test. Some Sine waves on audio output channels [0,1]
Ndef(\audiotest, {
	SinOsc.ar([1,2,3]*400).sum * 0.1;
}).play;
);
Ndef(\audiotest).clear; // destroy the synth


(

// OSC input mapping for Notey Patch
OSCdef(\esp32, {|msg|
	var gains = Array.newClear(13);
	msg = msg[2..];
	msg.join(":").postln;

	// Map all 13 sensor values to gain levels for each "note"
	13.do {|idx|
		var param = "v%".format(idx+1).asSymbol;
		gains[idx] = msg[idx].explin(50, 300, 0.0, 1.0, clip: \minmax); // map range 50:300 to 0.0:1.0 with exponential curve
		Ndef(\synthpatch).set(param, gains[idx]);
		//"% %".format(param, gains[idx]).postln;
	};
	gains.postln;
}, "/esp32");


);



( // Notey Vocoder Synth
Ndef(\synthpatch, {|pregain=10.0, gain=1.0|
	//var pitches = ([1,2,3,4,5,6,7,8,9,11,12,13] + 36).midicps; // definitely play with the pitch distribution to see what's possible
	var pitches = (Scale.major.ratios ++ (Scale.major.ratios[..5] * 2)) * 220; // simple major scale over 13 notes, starting at A3 (220hz)
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

	carriers = Saw.ar(pitches) * volumes;
	sig = Vocoder.ar(carriers, voicein, 20, 200, 5000, 0.02, 5000, 0.05, 25) * pregain;

	//sig = Splay.ar(sig); // spread voices across the stereo field
	sig = Pan2.ar(sig.sum, 0.0); // mix down to mono and pan how you like
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
	sig = SinOsc.ar(pitches) * volumes;
	sig = Pan2.ar(sig.sum, 0.0);
	Limiter.ar(sig * gain);
}).play;

);


(

~last_modestate = 0;
~current_mode = 0;
~current_intensity = 0;

// Control for FX Patch
OSCdef(\esp32, {|msg|
	var modevals = [msg[2], msg[3], msg[4]];
	var modestate = -1;
	var intensityslider = msg[4..];
	var intensity = intensityslider.maxIndex; // will be an intensity value from 0-9

	"INTENSITY: %   MODE: %".format(intensity, modestate).postln;


	if(modevals.sum < 40) {
		modestate = -1;
	} {
		modestate = modevals.maxIndex;
		if(modestate != ~last_modestate) { // User triggered a mode change
			~current_mode = modestate
		};
	};
	~last_modestate = modestate;

	if(intensityslider.sum > 50) { // threshhold below which slider touches are ignored..
		switch(~current_mode, // Set intensity parameter based on current mode
			0, { // Reverb
				// maybe try other params like \revtime or \revmix ... just make sure to map the intensity to sensible values
				var revsize = intensity.linlin(0, 9, 0.5, 100.0);
				Ndef(\synthpatch).set(\revsize, revsize);
			},
			1, { // Distortion
				var predist = intensity.linexp(0, 9, 1.0, 100.0);
				Ndef(\synthpatch).set(\predist, predist);
			},
			2, { // Delay
				var deltime = intensity.linlin(0, 9, 0.1, 1.0);
				var deldecay = intensity.linexp(0, 9, 0.2, 1.5);
				Ndef(\synthpatch).set(\deltime, deltime, \deldecay, deldecay); // Let's try manipulating two parameters at once!
			},
			{ "Unknown mode value!".error }
		);

	};

}, "/esp32");


);


( // FX Patch

Ndef(\synthpatch, {|gain=1.0| // This is an FX patch! We call it Notey to make switching between synths easier, no need to free previous notey!

	//var pitches = ([1,2,3,4,5,6,7,8,9,11,12,13] + 36).midicps;
	var revsig, distsig, delsig, mix;
	var voicein = SoundIn.ar(0);

	distsig = (voicein * \predist.kr(10.0, 0.1)).softclip;
	delsig = CombC.ar(voicein, 1.0, \deltime.kr(0.4, 0.1), \deldecay.kr(0.5, 0.1));

	mix = [voicein * \voicemix.kr(1.0, 0.1), distsig * \distmix.kr(1.0, 0.1), delsig * \delmix.kr(1.0, 0.1)].sum;
	revsig = GVerb.ar(mix, roomsize: \revsize.kr(10, 0.1), revtime: \revtime.kr(3.0, 0.1), damping: 0.5);

	mix = (mix * \noverbmix.kr(0.2, 0.1)) + (revsig * \revmix.kr(1.0, 0.1));

	Limiter.ar(mix * gain);

}).play;

);


