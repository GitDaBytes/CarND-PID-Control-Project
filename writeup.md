# Udacity PID Controller Project

The goal of this project was to implement a PID Controller in C++ to control a vehicle around a test track - in simulation. As the simulated car drives around the track, the vehicle feeds out the cross track error of the vehicle for each frame of the sim.

The cross track error (CTE) is the distance of the car's postion to the enter of the road. Using the CTE, the PID controller should return a steering input that will bring the car back into line with the intended trajectory (down the middle of the road).

The PID Controller is comprised of three components as the name suggests. *P*, *I*, *D*. A brief overview of the three components follow hereafter:

## Component P

The *P* component is arguably the main and heaviest hitting component of the full controller. Think of it as the force that will turn the car back towards the intended trajectory. The higher the CTE, the more *P* will turn the wheel of the car back towards the trajectory line. 

The trouble is, if we use the *P* component alone, there will be a tendancy (almost a certainty) that we will overshoot the intended trajectory line. Which will then cause P to respond by turning back to the target trajectory again, and so on. The effect is that we ended up with a 'wobble' around the trajectory line, which is not ideal for a passenger vehicle.

## Component D

*D* acts as a dampening on top of *P*. As before *P* turns the car back to the trajectory... but will overshoot the trajectory, as the vehicle approaches the trajectory we add in *D* to increase as the vehicle nears the target trajectory. *D* in effect lessens the impact of *P* as we approach the trajectory, thus dampening the effect of *P* when the CTE is low, bringing the vehicle onto line and hopefully eliminating the 'wobble'.

## Component I

Finally, the *I* component affects bias in the vehicle. The anaolgy given is that the vehicle wants to track constantly left or right (such as may be with a car with misaligned wheels / axel). The *I* component adds an opposite bias to negate that effect. 

## Project Implementation

The values that were selected for my implentation in this project were done so by trial and error. I went through an iterative loop whereby I set the numbers to an arbritary value, then would run the sim, monitoring the vehicles behavior. If I saw wobble in the car, I would tweak *D*, if the car did not turn a corner or return to a straight line fast enough, I tweaked *D*. I also, played with *I* which I found more difficult to set. It seemed that some values of I would stop the car turning tighter corners. In the end I opted to keep a trailing history of CTE values and compute I over that short history. This seemed to work better, allowing the car to turn the corner, while allowing turning bias to be counteracted. If I had more time to work on this project, I would have liked to try and optimize these parameters more. One option in particular I would have tried was the "twiddle" algorithm which iteritively tests the effect on *P*, *I* amd *D* on the vehicle position and optimizes the values of each to the optimal setting.