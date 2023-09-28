// ==== 1. The Absolute Basics: Starting the Server, Getting Help, Running Code ==== //

// one line comment
/*
Multi line comment
*/

// List your devices
ServerOptions.devices();
ServerOptions.devices.do({|dev,idx| postln("%:%".format(idx,dev)) }); // print them out one line at a time with index numbers

// Set an audio device to use
Server.default.options.device = ServerOptions.devices[12];

Server.default;

Server.killAll();

// Start the Synthesis Server
Server.default.boot();

// Make some sound
{ SinOsc.ar() * 0.1 }.play;

/*
SHORTCUTS:

CTRL+.         >> stop all sounds (you probably want to do this if you ran the line of code above!)

SHIFT+ENTER    >> evaluate the selected line, or whatever line the cursor is on
CTRL+ENTER     >> run all statements inside a block, wherever your cursor is...
CTRL+d         >> open help documentation for whatever keyword is selected
CTRL+i         >> look up source code for whatever class or method is selected
CTRL+SHIFT+L   >> recompile the class library / language / all commands you have run and memory state will be lost!
CTRL+b         >> start the default synthesis server

*/

// Make some sound (i.e. make a synth and play it) ... this time assign the playing synth to a variable x
x = { SinOsc.ar(300) * 0.2 }.play;

postln("Hello");
"Hello".postln;

String

"Hello".class;
120.class;
1.2.class;
{ }.class;

x = {
	(SinOsc.ar(300) + SinOsc.ar(800) + SinOsc.ar(1600)) * 0.05
}.play;

// Stop the synth from playing and free its memory
x.free;

// Quit the server..
Server.default.quit;

// NOTE: by default, SuperCollider sets the default server to the variable 's' for a little less typing.
//       so you can also start and stop the server like this...
s.boot;
s.quit;


// Often you'll want to run a bunch of statements all in one go. This can be done by making a block of statements with ( )
// You can run the blocks below by putting your cursor inside of them and running with CTRL+ENTER
(
x = { Saw.ar([200,300]) * 0.3 }.play;
y = { Pulse.ar([250,350]) * 0.3 }.play;
);

// Stop them both and start a new set with the following block.
(
x.free;
y.free;
x = { SinOsc.ar([200,300]) * 0.3 }.play;
y = { LFTri.ar([250,350]) * 0.3 }.play;
);

// Finally, stop them with the following block.
(
x.free;
y.free;
);

// Cool, now you're live coding.






// ==== 2. More Basics: Functions & Values  ==== //

// Functions are used all over the place in SuperCollider. They can be stored in variables and passed around
// just like numbers and strings. This is because everything (including functions) is an OBJECT in SuperCollider.
// You define a function using the format
{|argument1, argument2| /* your function code goes here */ }


def myfunc(arg1, arg2, arg3):
   postln(arg1)

{|arg1,arg2,arg3|
	postln(arg1);
}

// The most basic function you can imagine.
{ postln("Hello World") }.value();

f = { postln("Hello World") };
f.value();

// FUNCTIONS ARE VARIABLES!! -> the official way of saying this in CS "Functions are First Class Citizens"



// The most simple example of a function with no arguments... store it in the variable f to make using it easier.
// Then call the 'value' method to evaluate it. Note that strings in supercollider are written with "Double Quotes"
f = { postln("██████████ Hello World ██████████") };
f.value;

f = {|arg1, arg2|
	postln(arg1);
	postln(arg2);
};

// For default arguments, do this...
f = {|arg1="default1", arg2="default2"|
	postln(arg1);
	postln(arg2);
};


f.value();
f.value("Hello");
f.value("Hello", "World");

a = nil; // nil is SuperCollider's "nothing value"

// A more complex function that takes a string and an integer and prints the string that many times.
f = {|printme="default", thismanytimes=5|
	thismanytimes.do( { postln(printme) } );
};

f.value();

f.value("SuPeRcOlLiDeR", 10);

// do() is basically a for loop ... and you give it a function to do X times.

3.do( {|i| postln(i) } );

// The same as a for loop with i going from 0->2
do(3, {|i|
	postln(i)
});

// See how the function is treated as if it was any other kind of value?

// You can also give function arguments default values..
f.value(); // this prints 'nil' (SuperCollider's value for "nothing")
f = {|printme="Default", thismanytimes=3| thismanytimes.do({ postln(printme) }) }; // redefine with default values
f.value(); // Now your default values kick in when you don't specify any


// Functions that have sound signal generators in them (aka. UGens) are special.
// They can be sent to the server, which uses them to construct a synth
u = { RLPF.ar(Saw.ar([261,587]) * Saw.ar(8), LFTri.ar(3.7).range(40, 2200), 0.3) }; // a function with UGENS
x = u.play(); // play sends it to the server and tells the server to play it as soon as it can

x.run(false); // pause the synth, but don't free its memory from the server
x.run(true); // resume playing

x.free;

// Arguments in synth functions become parameters you can use to control synths in realtime.
u = {|f1=261,f2=587,lfo1=8,lfo2=3.7,rq=0.3|
	RLPF.ar(Saw.ar([f1,f2]) * Saw.ar(lfo1), LFTri.ar(lfo2).range(40, 2200), rq)
};
x = u.play();

// Change parameter values on the fly with the set method. Parameters are set using name,value pairs.
// The name is a special type of SuperCollider value called a Symbol ... you write a symbol either as \mysymbol or 'mysymbol'
x.set(\f1, 300);
x.set(\f1, 400);
x.set(\lfo1, 3);
x.set(\rq, 0.05);
// You can also set multiple parameters at once...
x.set(\f1, 774, \f2, 460, \lfo1, 4.6, \lfo2, 3.4, \rq, 0.1);
x.free;

// You refer to argument names by writing them as symbols:
Symbol
\myname
'myname'

Server


y = {|f1=261,f2=587,lfo1=8,lfo2=3.7,rq=0.3|
	RLPF.ar(LFTri.ar([f1,f2]) * Pulse.ar(lfo1), SinOsc.ar(lfo2).range(40, 2200), rq)
};
l = y.play();



// ==== 3. Control: GUIs ==== //

// Making GUIs in SuperCollider can be a little tedius, but they're useful to have sometimes.
// Especially since we don't have any sensor skins yet, it's helpful to be able to control audio from a gui.

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













