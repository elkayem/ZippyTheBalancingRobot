# ZippyTheBalancingRobot

![Zippy](/images/IMG_1361.JPG)

Zippy is an Arduino Nano based balancing robot.  The original prototype version was put together on a breadboard with lots of jumper wires.  This is certainly a great way to get started.  The STMicroelectronics 8A motor drivers do not come in a convenient package, but ST does provide evaluation boards which work well with a prototype system.

The latest version uses a PCB, allowing the motor drivers to be mounted directly to the board.  This yields a more compact design, but does require some SMD work and an additional outlay for PCB manufacture.  Even with the fine 0.5 mm pitch, the drivers can be hand soldered to the board (I did!) with a steady hand and some patience.  There are plenty of YouTube videos on how to do this.

Here are the important parts for Zippy
* 2 x VNH5180A motor drivers ($3.72 each as SMD parts, or $8.69 each as evaluation boards from Mouser)
* (Optional) Zippy printed circuit board (see Eagle files) $17.60 from EasyEDA or other PCB maufacturers
* 2 x 30:1 Metal Gearmotor 37Dx68L mm with 64 CPR Encoder ($39.95 each from Pololu)
* GY-521 MPU-6050 gyro/accelerometer breakout board ($4.78 from Amazon)
* Arduino Nano ($2.99 clone from Banggood)
* ZIPPY Flightmax 3000mAh 3S 20C LiPo battery ($22.49 from Amazon)
* 16x2 character LCD display ($5.99 from Amazon)
* Remote control with PWM receiver
* Two Schottky diodes (I used Fairchild SB1245 12A diodes, $1.57 each from Mouser)
* Various resistors, capacitors, 10K trimpot, pushbutton, connectors (see schematic)

I've also included a schematic in the Eagle folder.  Gerber files are provided in the Eagle folder if ordering from a PCB manufacturer.  Otherwise, use a breadboard and wire according to the enclosed schematic.

Pay close attention to polarity on motor wheels (swapping motor leads as necessary) and MPU-6050 orientation.  The software assumes the gyro/accelerometer is mounted with the +X axis facing the front of the robot.  

Depending on your robot inertia, the PID gains may need to be retuned.  Increase as needed to achieve crisp response without any oscillation.  

![Zippy](/images/IMG_1362.JPG)
<img src="/images/IMG_1352.JPG" alt="PCB" width="400" height="400"> <img src="/images/IMG_1356.JPG" alt="PCB Installed" width="533" height="400">

[![ZIPPY_VIDEO](https://img.youtube.com/vi/V53LkU0RIlw/0.jpg)](https://www.youtube.com/watch?v=V53LkU0RIlw) 


