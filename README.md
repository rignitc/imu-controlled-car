# imu-controlled-car
Node MCU Code for car controlled using IMU fixed on gloves.

The PWM values are calculated in one node MCU and then sent to the other using UDP protocol.

The node MCU with IMU acts as UDP client.
The other node MCU acts as UDP server.
