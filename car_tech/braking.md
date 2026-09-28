# Braking: geometry and a simple physics model

A brake needs both a rotating part attached to the wheel and a stationary reaction
path into the chassis. Experiment 003 adds drum-to-hub sleeves, backing plates,
anchors, a cross shaft and hand linkage. A floating drum near the wheel is not a
mechanical connection. Keep fixed parts out of the rotating wheel envelope and
check the full brake assembly with the wheels turning.

For a rolling vehicle on a slope, a useful approximate downhill acceleration is:

`a = (M*g*sin(theta) - sum(T_brake / wheel_radius)) / (M + sum(I_wheel / wheel_radius²))`

This assumes rigid wheels, no sliding, fixed steering, and no other resistance.
Use positive acceleration downhill. At rest, the brake capacity must cover the
gravity term; available tire/road friction also limits the force. Increasing brake
torque beyond the available grip produces sliding rather than unlimited stopping
force. A stopped wheel alone is therefore not evidence that the vehicle stopped.

A passive brake opposes angular motion and dissipates energy; its mechanical
power while slipping is `-T*abs(omega)`. At rest, static friction supplies only
the holding torque needed, up to its capacity. MuJoCo's joint `frictionloss`
provides this equivalent behavior; it does not resolve individual brake shoes.
See [MuJoCo's friction model](https://mujoco.readthedocs.io/en/latest/computation/).

Our tests measure chassis speed and travel as well as wheel rotation, then test
holding drift and release. Stopping distance begins at the command to apply the
brakes, including the 0.25-second application ramp. All masses, friction values
and torque capacity in this experiment are assumptions rather than historical
specifications. No thermal fading, wear, cable compliance or structural strength
has been modeled.
