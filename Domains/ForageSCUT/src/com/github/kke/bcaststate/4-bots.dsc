// This is an example description file for specifying the environment
// when using TBSim.


//======
// SIMULATION BOUNDARY
//======
//
// bounds left right bottom top
//
// bounds statements set the bounds of the visible "playing field" in
// meters for a simulation.   If the aspect ratio of the bounds are not
// the same as the graphical area set aside by the simulation, then
// the robots may wander off the screen.

bounds -5 5 -5 5


//======
// SEED
//======
//
// seed number
//
// The seed statement sets the random number seed.  The default is
// -1

seed 3


//======
// TIMESTEP
//======
//
// timestep milliseconds
//
// timestep statements set the time (in milliseconds) transpices
// between discrete simulation steps.

timestep 100 // 1/10th of a second

//======
// WINDOWSIZE
//======
//
// windowsize width height
//
// The windowsize statement gives a default window size.  This can be
// overridden on the command line.

windowsize 500 500


//======
// BACKGROUND COLOR
//======
//
// background color
//
// A background statement sets the background color for the simulation.
// The color must be given in hex format as "xRRGGBB" where RR indicates
// the red component (00 for none, FF for full), GG is the green component,
// and BB is the blue.  Here we use white:

background xFFFFFF


//======
// ROBOTS
//======
//
// robot robottype controlsystem x y theta forecolor backcolor
//              visionclass
//
// robot statements cause a robot with a control system to be instantiated
// in the simulation.  Be sure to include the full class name for the
// abstract robot type and your control system.  The x y and theta
// parameters set the initial position of the robot in the field.
// You can used different colors to tell robots apart from one another.
// The visionclass indicates which color the robots see each other as.

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim
	com.github.kke.bcaststate.ForageBcastState 2 2 0 xFF0000 x000000 2

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim
	com.github.kke.bcaststate.ForageBcastState 2 -2 0 xFF0000 x000000 2

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim
	com.github.kke.bcaststate.ForageBcastState -2 2 0 xFF0000 x000000 2

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim
	com.github.kke.bcaststate.ForageBcastState -2 -2 0 xFF0000 x000000 2

robot EDU.gatech.cc.is.abstractrobot.MultiForageN150Sim
	com.github.hy.simple.Escape 0  0 0 x000000 xFF0000 0

//======
// OBJECTS
//======
//
// object objecttype x y theta radius forecolor backcolor visionclass
//
// Pbject statements instantiate things without control systems (like
// balls, bins, obstacles, etc. Be sure to include the full class name for the
// object.  The x y and theta parameters set the initial position of
// object in the field.  Forecolor and backcolor are the foreground
// and background colors of the object as drawn. The visionclass
// parameter is used to put each kind of object into it's own perceptual
// class.  That way when the simulated sensors of robots look for things
// they can be sorted by this identifier.

// obstacles
object EDU.gatech.cc.is.simulation.ObstacleSim -2.0 -1.0 0 0.30 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 3.0 2.0 0 0.10 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 3.3 1.8 0 0.30 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim -4  4 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 4  -3.5 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 3.5 -2 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim -3.5 -2 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 3.5 -2 0 0.25 xC0C0C0 x000000 2
object EDU.gatech.cc.is.simulation.ObstacleSim 4.0 4.0 0 0.25 xC0C0C0 x000000 2
