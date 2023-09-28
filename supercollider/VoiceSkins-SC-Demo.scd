// ==== The Absolute Basics: Starting the Server, Getting Help, Running Code ====

// List your devices
ServerOptions.devices;
ServerOptions.devices.do({|dev,idx| postln("%:%".format(idx,dev)) }); // print them out one line at a time with index numbers

// Set an audio device to use
Server.default.options.device = ServerOptions.devices[15];

// Start the Synthesis Server
Server.default.boot;

// Make some sound
{ SinOsc.ar() * 0.1 }.play;

/*
SHORTCUTS:

CTRL+.         >> stop all sounds (you probably want to do this if you ran the line of code above!)

SHIFT+ENTER    >> evaluate the selected line, or whatever line the cursor is on
CTRL+d         >> open help documentation for whatever keyword is selected
CTRL+i         >> look up source code for whatever class or method is selected
CTRL+SHIFT+L   >> recompile the class library / language / all commands you have run and memory state will be lost!
CTRL+b         >> start the default synthesis server

*/

// Make some sound (i.e. make a synth and play it) ... this time assign the playing synth to a variable x
x = { SinOsc.ar(300) * 0.2 }.play;

// Stop the synth from playing and free its memory
x.free;

// Quit the server..
Server.default.quit;

// NOTE: by default, SuperCollider sets the default server to the variable 's' for a little less typing.
//       so you can also start and stop the server like this...
s.boot;
s.quit;
