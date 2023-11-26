/*
BLEND EXPERIMENT INSTRUMENT

FADER1 - Loudness
FADER2 - Pitch
FADER3 - Vibrato
FADER4 - Resonance/Vowel
FADER5 - Air/Bigness/Harmonics
FADER6 - Human/Synthetic

*/


// STARTUP
(

// Adjust to your PC's hardware capabilities...
s.options.blockSize=512; // A low block size in scsynth is fine
s.options.hardwareBufferSize=512; // This should match RAVE match hardware...
s.options.memSize=65536 * 4;


s.waitForBoot {
	w = ();
	w.m = Array.newClear(4); // model synths array
	//w.modelids = [\jvox95, \blonk2_95, \chorala2, \animal2]; // the ids of the four models
	w.modelids = [\jvox95, \blonk2_90, \chorala2, \acryvae]; // the ids of the four models
	//w.modelids = [\musilon, \musilonfront, \chorala2, \animal2]; // the ids of the four models

	{
		w.meter=s.meter;
		1.wait;
		w.meter.window.bounds = Rect(Window.screenBounds.width, Window.screenBounds.height, 320,230);
	}.fork(AppClock);

	~use_models = [\animal2, \blonk2_90, \chorala2, \jvox95, \vctk1k, \acryvae, \nasa];

	w.modelspecs = RAVEModelManager.parseModelspecFileOrString(jsonfile: "NN_Modelspec.json".resolveRelative, use_models: ~use_models);

	// Load RAVE Models for NN.ar
	w.modelspecs.keysValuesDo {|key,mspec| NN.load(key, mspec[\path]) };

	~ravegroup = Group.new(s, \addToHead);
	~srcgroup = Group.new(s, \addToHead);
	~mastergroup = Group.new(s, \addToTail);
	~fxgroup = Group.new(s, \addToTail);

};

);

/* Testing NN Tilde
NN.describeAll;
*/

// Build synths

( // Source... can be other signals
w.src1.free;
w.src1 = play( {|pitch=250,vibrato=0.0,loudness=1.0,air=0.0,formant_pitch=1500,wave_mix=0.0|
	var sig, vib, tri, saw, formant, wave;
	vib = SinOsc.ar(5);
	tri = LFTri.ar(pitch + (vibrato * vib.range(-10,10)));
	formant = Formant.ar(pitch, pitch + formant_pitch, 800);
	wave = XFade2.ar(tri, formant, wave_mix);
	//sig = SinOsc.ar(LFNoise1.ar(0.5).range(300,2000)) * LFNoise2.ar(1).range(0.05, 0.3);
	//sig = (SinOsc.ar(MouseY.kr(100, 2800)) + (WhiteNoise.ar() * MouseX.kr(0.0, 0.1))) * LFNoise2.ar(0.5).range(0.00, 0.16);
	sig = wave + BBandPass.ar(WhiteNoise.ar * air, 7000, 0.4);
	sig = sig * loudness;
	//sig = BLowPass4.ar(sig, MouseX.kr(50, 8000), 0.4) * MouseY.kr(0.0, 0.8);

	//sig = LFTri.ar(LFNoise1.ar(0.3).range(500,3000)) * EnvGen.ar(Env.perc, Impulse.ar(LFNoise1.ar(0.5).range(0.5,5)), timeScale: LFNoise1.ar(1).range(1.0, 3.0)) * LFNoise2.ar(2).range(0.01, 0.1);

	//sig = Mix([sig, sig1]);
	sig = Limiter.ar(sig.tanh);
	sig;
}, outbus: 28, target: ~srcgroup);
);
//w.src1.free;




( // CREATE MODEL SYNTHS...


( // Don't put multiple RAVE Ugens in the same synth for some reason... or do? This just weirdly explodes sometimes!
w.m[0].free;
w.m[0] = play( {|pan=0.0, gain=1.0|
	var model=w.modelids[0];
	var numLatents=w.modelspecs[model][\numLatents];
	var res,sig,z;
	var zscale = numLatents.collect{ |i| ("s"++i).asSymbol.kr(1.0) };
	var zbias = numLatents.collect{ |i| ("b"++i).asSymbol.kr(0.0) };
	//var znoise = numLatents.collect{ |i| ("n"++i).asSymbol.kr(0.0) };
	sig = In.ar(28);
	sig = Compander.ar(sig, sig, 0.01, 1.5, 1.0);
	//res = NN(model, \forward).ar(sig, warmup: 1); // should be slightly faster to use forward vs. split
	z = NN(model, \encode).ar(sig, warmup: 1);
	z = (z * zscale) + zbias;
	res = NN(model, \decode).ar(z, warmup: 1);
	Pan2.ar(res, pan, gain);
}, outbus: 30, target: ~ravegroup);
);
//w.m[0].free;


(
w.m[1].free;
w.m[1] = play( {|pan=0.0, gain=1.0|
	var model=w.modelids[1];
	var numLatents=w.modelspecs[model][\numLatents];
	var res,sig,z;
	var zscale = numLatents.collect{ |i| ("s"++i).asSymbol.kr(1.0) };
	var zbias = numLatents.collect{ |i| ("b"++i).asSymbol.kr(0.0) };
	//var znoise = numLatents.collect{ |i| ("n"++i).asSymbol.kr(0.0) };
	sig = In.ar(28);
	//sig = Pulse.ar(400) * LFNoise2.ar(7).range(0.2, 0.7);
	//res = NN(\blonk2_95, \forward).ar(sig, warmup: 1); // should be slightly faster to use forward vs. split
	z = NN(model, \encode).ar(sig, warmup: 1);
	z = (z * zscale) + zbias;
	res = NN(model, \decode).ar(z, warmup: 1);
	Pan2.ar(res, pan, gain);
}, outbus: 32, target: ~ravegroup);
);
//w.m[1].free;


(
w.m[2].free;
w.m[2] = play( {|pan=0.0, gain=1.0|
	var model=w.modelids[2];
	var numLatents=w.modelspecs[model][\numLatents];
	var res,sig,z;
	var zscale = numLatents.collect{ |i| ("s"++i).asSymbol.kr(1.0) };
	var zbias = numLatents.collect{ |i| ("b"++i).asSymbol.kr(0.0) };
	//var znoise = numLatents.collect{ |i| ("n"++i).asSymbol.kr(0.0) };
	sig = In.ar(28);
	//res = NN(\chorala2, \forward).ar(sig, warmup: 1); // should be slightly faster to use forward vs. split
	z = NN(model, \encode).ar(sig, warmup: 1);
	z = (z * zscale) + zbias;
	res = NN(model, \decode).ar(z, warmup: 1);
	res = JPverb.ar(res, 2.0, 0.4, 2.0, 0.3, 0.4, 2.0) + res;
	res * gain;
}, outbus: 34, target: ~ravegroup);
);
//w.m[2].free;

(
w.m[3].free;
w.m[3] = play( {|pan=0.0, gain=1.0|
	var model=w.modelids[3];
	var numLatents=w.modelspecs[model][\numLatents];
	var res,sig,z;
	var zscale = numLatents.collect{ |i| ("s"++i).asSymbol.kr(1.0) };
	var zbias = numLatents.collect{ |i| ("b"++i).asSymbol.kr(0.0) };
	//var znoise = numLatents.collect{ |i| ("n"++i).asSymbol.kr(0.0) };
	sig = In.ar(28);
	//sig = Pulse.ar(400) * LFNoise2.ar(7).range(0.2, 0.7);
	//res = NN(\animal2, \forward).ar(sig, warmup: 1); // should be slightly faster to use forward vs. split
	z = NN(model, \encode).ar(sig, warmup: 1);
	z = (z * zscale) + zbias;
	res = NN(model, \decode).ar(z, warmup: 1);
	Pan2.ar(res, pan, gain);
}, outbus: 36, target: ~ravegroup);
);
//w.m[3].free;


( // Catch and blend four models in an X/Y configuration from -1 to 1 in 2D
w.morph.free;
w.morph = play( {|gain=1.0, srcmix=0.0, ravemix=1.0, mix_x=(-1.0), mix_y=1.0|
	var src,res,verb,del,sigs,jonjaap,choranimal,mix;
	src = In.ar(28, 1);
	src = DelayC.ar(src, 0.2, \makeupdelay.kr(0.1));
	sigs = In.ar([30,32,34,36], 2); // four models to mix
	jonjaap = XFade2.ar(sigs[0], sigs[1], mix_x);
	choranimal = XFade2.ar(sigs[2], sigs[3], mix_x);
	res = XFade2.ar(choranimal, jonjaap, mix_y);
	mix=Mix([res*ravemix,src*srcmix]);
	del = DelayC.ar(mix, 0.2, 0.1, 0.3);
	//verb = GVerb.ar(mix, 100, 2, 0.1, 0.1, 10, 0.0, 0.2, 0.1, 400);
	verb = JPverb.ar(mix, 2.0, 0.4, 3.0, \earlyref.kr(0.8), 0.05, 2.0, 0.7, 0.1, 0.7, 300, 2000);
	mix=Mix([verb*\verbmix.kr(0.4), mix, del*\delmix.kr(0.1)]);
	mix * gain;
}, outbus: 0, target: ~fxgroup ); // TODO: wish to be able to send to master, an fx path, or designate as an fx synth on a different group..
);

w.morph.set(\srcmix, 0.0, \ravemix, 1.0);
w.morph.set(\earlyref, 0.3, \verbmix, 0.3, \delmix, 1, \makeupdelay, 0.15);


// Reset all latents...
w.resetLatents = {|e, bias=0.0, scale=1.0|
	4.do {|idx|
		var modelsynth = w.m[idx];
		8.do {|lat|
			modelsynth.set(("b"++lat).asSymbol, bias);
			modelsynth.set(("s"++lat).asSymbol, scale);
		};
	};
};


);







// SETUP MIDI
(



m = ();

m = NanoKontrol2.new(initmidi:true);

);



(

// Setup Nanokontrol2

// General Controls over Morphing between models
m.setcb(6, "knob", {|val, spec|
	val=val.linlin(0.0, 1.0, -1.0, 1.0);
	w.morph.set(\mix_x, val);
	//{m.gui.kmix34.value = val}.fork(AppClock); // this is handled within the NanoKontrol class
});
m.setcb(7, "knob", {|val, spec|
	val=val.linlin(0.0, 1.0, -1.0, 1.0);
	w.morph.set(\mix_y, val);
	//{m.gui.kmix12.value = val}.fork(AppClock);
});
m.setcb(6, "fader", {|val, spec|
	postln("Set src: %".format(val));
	w.morph.set(\srcmix, val);
	//{m.gui.ksourcelevel.value = val}.fork(AppClock);
});
m.setcb(7, "fader", {|val, spec|
	postln("Set ravemix: %".format(val));
	w.morph.set(\ravemix, val);
	//{m.gui.kmixall.value = val}.fork(AppClock);
});




/*
Loudness/Dynamics 1
*/
m.setcb(0, "fader", {|val, spec|
	val=val.linlin(0.0, 1.0, 0.0, 1.0);
	w.src1.set(\loudness, val);
}, true);


/*
Pitch 2
*/
m.setcb(1, "fader", {|val, spec|
	val=val.linlin(0.0, 1.0, 80, 340);
	w.src1.set(\pitch, val);
}, true);

/*
Vibrato 3
*/
m.setcb(2, "fader", {|val, spec|
	val=val.linlin(0.0, 1.0, 0.0, 0.8);
	w.src1.set(\vibrato, val);
}, true);

/*
Resonance/Vowel 4
*/
m.setcb(3, "fader", {|val, spec|
	var fp, wm;
	fp=val.linexp(0.0, 1.0, 300, 1000);
	wm=val.linlin(0.0, 1.0, -1, 1);
	w.src1.set(\formant_pitch, fp);
	w.src1.set(\wave_mix, wm);
}, true);



/*
Air 5
*/
m.setcb(4, "fader", {|val, spec|
	var ar=val.linlin(0.0, 1.0, 0.0, 0.5);
	var mx=val.linlin(0.0, 1.0, -1.0, -0.8);
	var my=val.linlin(0.0, 1.0, 1.0, 0.5);
	w.src1.set(\air, ar);
	w.morph.set(\mix_x, mx);
	w.morph.set(\mix_y, my);
}, true);

/*
Human/Synthetic 6
*/
m.setcb(5, "fader", {|val, spec|
	var ms=val.linlin(0.0, 1.0, 0.0, 1.0);
	var mr=val.linlin(0.0, 1.0, 1.0, 0.0);
	w.morph.set(\srcmix, ms);
	w.morph.set(\ravemix, mr);
}, true);



/*
w.resetLatents(0.0, 1.0);
*/

);


// CREATE GUI
(
{
	m.makeGUI(w.modelids);
	2.wait;
	m.assignControl.value(0);
	m.gui.knobs[6].valueAction_(-1); // X morph left
	m.gui.knobs[7].valueAction_(1); // Y morph top
	m.gui.sliders[6].valueAction_(0.0); // 0.0 src gain
	m.gui.sliders[7].valueAction_(1.0); // 1.0 rave gain

}.fork(AppClock);
);