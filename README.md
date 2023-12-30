# TODO:

 - [ ] Zips ties that keep the controllers on the modules make a clicking sound (I think?),
       might be worth looking into before it becomes a problem.
 - [x] Add feature to use analog absolute encoder for initial position and then switch to
       the NEOs built in encoders for (hopefully) better latency and redendency.
    - This will also let us use the built in PID controller taking more strain off the RoboRIO
    - (NEEDS TESTING) Is this worth it? The high current could be causing noise and the poor
      pin connector could fall out mid match.
 - [ ] Attach analog absolute encoder to the SPARK MAX
    - This will help clean up some of the wiring and improve noise (shorter wire).
    - OFFICIAL BOARD: https://www.revrobotics.com/rev-11-1278/ (built in 5v to 3.3v converter).
    - Probably worth it to design our own for cheaper.
 - [ ] Finish multi-camera pose estimation
    - This probably works already, just havn't gotten around to setting it up and finishing
      the pose estimation code.
    - NoodleVision needs a code clean up and better NT interface naming scheme.
 - [ ] Implement addressable RGB LED sub system
    - Attempt a 2D color mapped array so that as the robot enters and leaves areas of the field
      the colors will changed dynamically mapped to their physical positions.
    - Maybe 2D pixel wall? (5M * 60led/m = 300leds -> 15x15? maybe buy more? maybe higher desity?)
    - Very over the top, I like RGB.
 - [ ] Co-processesors power situation
    - Power needs to be measured, but each Orange Pi 5 consumes max 15ish watts (3 amps @ 5V)?
    - No idea if we will use the limelight, but those LEDs drain A LOT.
    - If machine learning ends up getting hooked up the raspberry pi 4 draws about 12ish watts,
      the pi 5 15ish watts, **coral consumes 900mA at full power (ugh, higher then rpi5 supports, kind of?)**
 - [ ] Add smart dashboard options for configuring different modes
    - Make the whole design code based, no more desiging in smart dash board since it keeps breaking.
 - [x] Implement PIDF for driving to obtain better speed correction
    - This needs better tuning, might not be worth it since it causes overshoot
 - [x] Switch to 2024 beta
 - [ ] Rename loggers to be better organized
 - [x] Switch to polar cordinates for the drive system to get around deadband along both x and y axis
