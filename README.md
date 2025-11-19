# Ball and Beam System
Jorge Larach
_11/18/25_

This is my repository for my ball and beam system. Here you will find firmware for an STM32 Nucleo that allows it to interface with a distance sensor and a servo to control the position of a ball along the surface of a beam.

You will also find in the "Notes" directory plots that I used to measure PID performance, written in Python using Matplotlib, part drawings from Fusion, as well as more pictures. 

<img src="./Notes/Photos/Nov 18 Photo 1.jpg"
     alt="PID Ball and Beam Project"
     style="width:100%; height:auto%;">

Below are all of the parts that I used making this project:
* STM32 Nucleo-F401RE
* HC-SR04 Ultrasonic distance sensor
* MG996R Servo
* 6V 2000 mAh NiMH battery
* Small Breadboard
* 3D printed structure (tower, beam, etc.)
* M3 nuts, bolts, and washers
* 1 ping pong ball

Here's a few test results plotted out in Matplotlib. Below is what my first (working) attempt looked like
<img src="./Notes/PID Tuning Analysis/Version 1/version1graph.png"
     alt="Version 1"
     style="width:100%; height:auto%;">

Below is the version currently used by the algorithm. It is definitely far from perfect, but it is a major improvement over the first version (nearly halved the MSE!) There's far less oscillations, but it looks like there's potentially some steady-state error, where the ball approaches the setpoint early on, but seems to hover underneath it for a while.
<img src="./Notes/PID Tuning Analysis/Version 3/version3graph.png"
     alt="Version 3"
     style="width:100%; height:auto%;">

I carved the groove in the beam from a 4 cm diameter circle in Fusion, so it perfectly fits any ping pong ball. My Bambulabs A1 printer couldn't print a beam longer than 25.6 cm due to the print bed size, so I designed it in two steps, consisting of a long bridge part and a short bridge part (during development I decided it would be called a "bridge" instead of "beam"; I don't know exactly what posessed me to do that) using two cylindrical pegs. Below is the Fusion drawing for the long bridge part:
<img src="./Notes/Fusion Drawings/Ball and Beam Drawing - Long Bridge Part.png"
     alt="Fusion drawing for long bridge part"
     style="width:100%; height:auto%;">

These drawings aren't strictly necessary for anyone interested in replicating my design (all STL files are also included in this repository), but it might give some insight into how all these parts are assembled.
Even though I feel satisfied with the current version, I would like to return to this project someday to address some issues and further improvements, such as:
* The surface of the beam/bridge might be too rough, and it might be causing the ball to slow down or get stuck in places.
* Pretty brutal amounts of sensor noise, despite sample smoothing using a ring buffer.
* The PID performance worsens as the setpoint approaches either edge of the beam; it seems to perform best near the center, at 20 cm.
* The algorithm itself also isn't perfect. I would like to add saturation checking, and keep tuning the gains to obtain better performance.
* I would like to add a second distance sensor next to the tower which sets the setpoint for the algorithm with a cube. That way, I can change the setpoint by moving the cube.
* Circular extender for servo arm. Right now, the bridge only moves +1 cm when servo is at 180 degrees, and -1 cm for 0 degrees. Maybe a larger range of motion improves PID performance?

When I return to this project, it will likely be after my next project, which I'm researching now. I would love to one day build a drone, but I think the most realisting next project I can acheive is a 3 axis robotic arm. Or who knows, maybe I'll find something else that blows my socks off. Overall, though, I'm really happy with where this project is at today. Further work will be detailed in this readme.
