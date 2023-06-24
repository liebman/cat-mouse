# Prototype Rover

Simple autonomous differential drive rover with LIDAR in RUST using an esp32s3.

- [motor](motor) contains a simple PWM motor control for cheap brushed motors using a phase/enable type motor controller.
- [encoder](encoder) contains a PCNT based quadrature encoder controller to track the position of the encoder.
- [speed-control](speed-control) implements PID control of motor speed using encoder.
- [position-control](position-control) allows the motor position to be controlled by specifying the wanted position in degrees.
- [wheel](wheel) is an abstraction on top of position-control to specify motor position in millimeters.
- [differential-drive](differential-drive) implements rover rotation and forward/reverse movement by specifying the distance or rotation.
- [proximity](proximity) is just a trait for proximity sensors
- [luna](luna) contains a driver for an [i2c based IR time-of-flight](https://www.amazon.com/TF-Luna/dp/B086MJQSLR) proximity detector.
- [lidar](lidar) implements LIDAR with a speed-controller and a proximity sensor by rotating the proximity sensor at a known degrees/second rate.
- [rover](rover) runs around autonomously avoiding things.
