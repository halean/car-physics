# Driving controls: how simulators show and take throttle and brake

Research behind the shared controls of the web simulator
([experiments/web](../experiments/web/README.md)), gathered on 2026-09-29.

## What the period cars actually had

The cars in this repository did not have a modern accelerator pedal:

- Their engines ran against a governor, or were set with a **hand throttle**. On
  the Model T this is a lever on the right of the steering column, beside the
  spark lever on the left. The Model T's three pedals are low/clutch, reverse and
  brake [C1].
- **Brakes were often split** between a foot brake working through the driveline
  (the Velo's band, the Mercedes' water-cooled countershaft drum, the Model T's
  transmission brake) and a lever working drums or bands at the rear wheels. The
  Panhard had only a rim-block lever. The Renault's single pedal declutched and
  then braked (see each experiment's README).

So the shared controls are a **hand throttle that stays where it is set**, a
**momentary brake pedal** and a **latching brake lever**. Each car maps the pedal
and the lever onto its own brakes, at the capacities its validation used. A car
without one of them says so.

## How driving simulators display inputs

- **Pedal bars.** Sim-racing overlays show live throttle, brake and clutch as
  bars, conventionally green for throttle and red for brake [C2, C3].
- **Pedal traces.** A rolling graph of throttle and brake over the last few
  seconds is the standard tool for reading technique, such as overlap and trail
  braking [C2, C4]. The garage shows both, plus the brake lever in brass.
- **Steering, gear and speed** sit beside the pedals in the same overlays [C3].
  The garage turns a wheel icon by the column angle times the car's box ratio,
  so the Model T's 4:1 and the Mercedes' 6:1 show at the wheel.

## How games take the input

- **Keyboard.** Keys are digital, so browser games ramp them: an "up/down"
  pair becomes an analog axis with attack, release and auto-centring [C5]. The
  garage ramps the brake pedal (on 3/s, off 5/s), the steering (1.4/s, centring
  at 2.8/s) and the hand throttle (0.8/s open, 1.2/s close).
- **Gamepad.** The Gamepad API is polled every frame. The triggers are analog,
  0 to 1, and should be read as `button.value`, not as pressed or released.
  Sticks need a deadzone, or a worn stick makes the car drift [C5, C6]. The
  garage maps the right trigger to throttle, the left trigger to the brake
  pedal and the left stick to steering, with a 0.12 deadzone.
- **Touch.** On-screen buttons lose the analog control that triggers give;
  that is the main complaint about touch racing controls [C7]. Games put gas
  and brake in the bottom corners, and better ones reduce how much the player
  has to hold [C8]. The garage's on-screen pedals read pressure from where the
  finger is on the pedal (higher is harder), which keeps them analog. The hand
  throttle means nobody has to hold a gas button.
- **One action layer.** Keyboard, gamepad and touch all feed the same named
  actions (throttle, pedal, lever, steer, gear, restart) [C5]. The car adapter
  then decides what each action does on that car.

## Sources

- **C1** Curbside Classic, [Driving impressions: Ford Model T](https://www.curbsideclassic.com/blog/driving-impressions/driving-impressions-ford-model-t-these-are-not-the-controls-you-are-used-to/);
  Model T Ford Fix, [How to drive a Model T Ford](https://modeltfordfix.com/how-to-drive-a-model-t-ford/).
- **C2** fteodoro803/sim-racing-telemetry, [Pedal Trace widget](https://github.com/fteodoro803/sim-racing-telemetry/pull/20);
  [SimTrace](https://github.com/LinyL4/SimTrace).
- **C3** [ACE Input Telemetry SimHub overlay](https://www.overtake.gg/downloads/ace-input-telemetry-simhub-overlay.81702/);
  [bo2 official overlays](https://github.com/fixfactory/bo2-official-overlays).
- **C4** MySimRig, [Sim racing telemetry for beginners](https://mysimrig.nl/en/blog/simracing/sim-racing-telemetry-for-beginners/).
- **C5** [Input: first-class browser gamepad controls for driving](https://github.com/je55pr/rtao/issues/93);
  JSGuides, [Using the Gamepad API](https://jsguides.dev/guides/javascript-gamepad-api/).
- **C6** Smashing Magazine, [Using the Gamepad API in web games](https://www.smashingmagazine.com/2015/11/gamepad-api-in-web-games/).
- **C7** The Race, [Is there room for a realistic mobile racing game?](https://www.the-race.com/gaming/is-there-room-for-a-realistic-mobile-racing-game/).
- **C8** Mobile Free To Play, [Touch control design: less is more](https://mobilefreetoplay.com/touch-control-design-less-is-more/).
