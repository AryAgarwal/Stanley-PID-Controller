# Stanley-PID-Controller
As a memebr of the Controls division of the Driverless subsystem at Formula Manipal, Icreated the Stanley and PID Controller.
# Pure pursuit Controller
Pure pursuit is the geometric path tracking controller. A geometric path tracking controller is any controller that tracks a reference path using only the geometry of the vehicle kinematics and the reference path. Pure Pursuit controller uses a look-ahead point which is a fixed distance on the reference path ahead of the vehicle as follows. The vehicle needs to proceed to that point using a steering angle which we need to compute.
The pure pursuit controller is a simple control. It ignores dynamic forces on the vehicles and assumes the no-slip condition holds at the wheels. Moreover, if it is tuned for low speed, the controller would be dangerously aggressive at high speeds. One improvement is to vary the look-ahead distance 𝑙𝑑 based on the speed of the vehicle.
# Stanley Controller
It is the path tracking approach used by Standford University’s Darpa Grand Challenge team. Different from the pure pursuit method using the rear axle as its reference point, Stanley method use the front axle as its reference point. Meanwhile, it looks at both the heading error and cross-track error. In this method, the cross-track error is defined as the distance between the closest point on the path with the front axle of the vehicle.
Firstly, eliminating the heading error. 𝛿 (t)= 𝜓(𝑡)
Secondly, eliminating the cross-track error. This step is to find the closest point between the path and the vehicle which is denoted as e(t). The steering angle can be corrected as follows,

The last step is to obey the max steering angle bounds. That means 𝛿(𝑡)∈ [𝛿𝑚𝑖𝑛,𝛿𝑚𝑎𝑥].
So we can arrive,

One adjustment of this controller is to add a softening constant to the controller. It can ensure the denominator be non-zero.

