# alex

Project ALEX involves creating a robot to navigate through an unknown terrain remotely with the help of the RPLIDAR, as well as to identify "victims" in the form of different colours. The project demands the technical application of both hardware and software design to achieve its deliverables.

The robot utilises two microcontrollers in its operation, the ARDUINO UNO and RASPBERRY PI (RPi). Both microcontrollers communicate serially with each other to govern the overall operation of the robot. In particular, the ARDUINO microcontroller powers and controls the motors of the robot, and the colour sensor to detect "victims". On the other hand, the RPi processes RPLIDAR data and communicate with the ARDUINO to create desired robot movements via user commands.

Finally, the RPi is connected to a remote laptop via TCP/IP connection with TLS. This allows for the wireless real-time connection between the laptop and RPi to receive the RPLIDAR's data to map the terrain, and transmit movement commands. With the aid of the SLAM algorithm, the terrain can be mapped remotely, allowing the robot to be controlled remotely.

Ultimately the project involves designing a complex computer engineering system that facilitates information processing, real-world interfacing, and understanding the effects of certain useful metrics such as, scaling, safety, security, sustainability, societal impact, fault-tolerant design, etc.
