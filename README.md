# Solar Concentrator

**A benchtop 2-axis tracking heliostat apparatus/solar concentrator for conceptual demonstrations of novel geoengineering innovations**

Mohan Hathi, Chris Stokes, Ye Tao
[Project MEER:ReflEction](https://www.meerreflection.com)
Tao Lab @ The Rowland Institute at Harvard University

![Overall Image](https://github.com/MohanMaker/Solar-Concentrator/Photos-Videos/1. Overview.jpeg)

![Movement Demo](https://github.com/MohanMaker/Solar-Concentrator/Photos/Demo_1.gif)

## Project Overview
As part of a project working on climate change solution with mirrors, we have created a small scale benchtop sun-tracking solar concentrator, which can be used to cook food, generate electricity, or in this case, calcinate calcium carbonate (clamshells) in a reaction chamber, as part of an ocean deacidification and CO2 capture cycle. The device is designed to be created out of common parts, and most of the components can be 3D printed, laser-cut, or CNC machined.

The system consists of two essential components, the mirror array reflector, and the electronic sun tracking system. The mirror array consists of a multitude of small mirrors (in this case 100) arranged in a rectangular grid formation. Each mirror is mounted on an individually angled 3D printed block, such that they all work together to concentrate light to one focal point, currently configured to be in front and above the mirror array. This is essentially like a custom parabolic mirror. In the Code > Mirror Mount Angles section of this repository, we provide code that enables all of these parameters to be customized. In the "Angle-Corner-Height-Calculations" script, you can input parameters such as the mirror dimensions, array size, and focal point position, and get a .csv file of angles and "corner heights" for each mirror mount block. Using the "Import-Corner-Heights-Fusion-Script" you can automatically customize the parametric Fusion 360 file for the mirror mount with the exported data, generating all of the unique 3D printable mount design files to build the array.

The second part of the system, the sun tracking electronics, move the entire array of mirrors on two axes to maintain a desired focal stationary point as the sun moves across the sky. A large bearing enables rotation of the entire system (azimuth control), while smaller bearings placed on top of two poles connect the system to the mirror array and allow for tilt (elevation control). These mechanical components are controlled using two geared stepper motors linked to a microcontroller running CircuitPython (a variant of python for microcontrollers). The microcontroller receives inputs from two sensors, a real-time clock (which enables precision time measurement), and an IMU or Inertial Measurement Unit (which allows the system to find its absolute position and tilt in space). The code running on the microcontroller (found in Code > Tracking Motion Control) takes the current time and location, and finds the azimuth and elevation angles of the sun using Astronomical Algorithms by Jean Meeus (provided by NOAA). It then calculates the desired position of the mirror array (remember, angle of incidence = angle of reflection across a normal line), and after comparing this position to the current position from the IMU, moves the solar tracker accordingly.

All of these parts come together to form a robust and versatile sun-tracking platform. Each component can be easily customized for better results: the mirror array can be changed to focus light to a custom focal point, and the solar tracking system can be modified to accept different types of mirror arrays, such as a commercial parabolic mirror or fresnel lens. In its current form, a 10 x 10 array of 2.5" mirrors, testing shows that the device can generate temperatures up to 80C, however, this can likely be improved upon in a variety of ways.

## Design Process
**Summer 2020:** The original design was based on a heliostat apparatus patent by Jesse Bunch. The design used an array of individually angled mirrors, similar to the current one, where each mirror was attached with a universal joint into a 2 layer grid. By moving the top layer of the grid while keeping the bottom stationary, the mirrors could be angled on two axes to track the sun. This design is advantageous because it can be incredibly compact - each mirror is moved individually. We built a 2 x 2 (4 mirror) array using this system. We experimented with different types of 3D printable universal joints that were durable, flexible, and prevented twisting, as well as with ways to fine-tune the mirror angles with set screws after assembly to achieve a tighter focal point. We also devised a system to slightly curve each mirror while attaching it to the mirror mount, which further reduced the focal size. After increasing to a 5 x 5 mirror array (25 mirrors), we found that this design became impractical due to a large number of moving parts (each mirror is moving by itself), and inaccuracy in the grid made it difficult for the mirrors to be perfectly aligned and to move together. Additionally, it was difficult to develop a system to link the stepper motors to the movement layer grid. Even when using four stepper motors (instead of two), there was significant play in the system.

**Summer 2021:** For the next iteration, we retained the best aspects from the original design while reducing the complexity of the system. Instead of having each mirror move individually to focus the light, we created an array of individually angled but stationary mirrors mounted to a large pegboard. The entire array of mirrors could then be moved as a whole to focus the light. This eliminated the need for an imprecise grid and universal joints. We created numerous iterations of the mirror mount blocks to create an array with the tightest possible focal point. The final design uses blocks that are screwed to the mirror and the pegboard with two connection points, creating a tight fit that prevents rotation. Despite this, we found that the focal point was still not very tight (only around 6 - 9 inches), due to inherent imprecision in manufacturing. The elevation and azimuth control system is based on concepts from simple solar panel trackers found on YouTube. To get precise yet strong movement control from the stepper motors, we experimented with a variety of gearboxes and gearing systems. The code was significantly re-written to improve the accuracy of solar angle calculations and to work with the new tracking system. Additionally, we added an IMU sensor so that the device can find its absolute position in space. This eliminated user error in setup and improved tracking accuracy.

**Future Considerations:** Despite somewhat low temperatures (80C) in our testing, this is still a powerful system, and some improvements and modifications could significantly improve heat output by reducing the focal size. Ideas include using a parabolic mirror or Fresnel lens instead of the custom mirror array to achieve a tighter focal point and higher efficiency. 



For Project Administrators Only:
(Google Drive Documentation)[https://drive.google.com/drive/folders/1jxcG3tS0jDyiT93cw37qHE2v5bExMBay?usp=sharing]