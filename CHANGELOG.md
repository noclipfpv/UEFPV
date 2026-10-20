# Changelog

Plugin for **UEVR 1.05** (plugin API 2.34), flat screen.

## v0.7.7
- RC radio detection: connected joysticks / radios listed in F6 → Input with live axis values,
  click to select; the saved id is fixed up once at start if the radio moved to another id;
  winmm device list refreshed so a radio plugged in after the game started shows up.
- Hint in the menu when no radio is visible (Steam Input hides controllers).

## v0.7.5
- Inspire FPV camera: the tilt-only stabilizer reacts slightly late (short dip / bump when
  accelerating and braking), adjustable "FPV tilt stabilizer delay" (0 = perfectly stable).

## v0.7.4
- Flying wing camera tilt 0 deg by default.

## v0.7.3
- Inspire FPV camera stabilized on tilt only (0 deg by default), heading and roll follow the
  aircraft; softer body lean ("Body tilt smoothness").

## v0.7.2
- Dual OP live: camera operator on a gamepad (own deadzone) or an RC radio (axis mapping,
  inversion, calibration, zoom as focal length or zoom speed, deadzone 0 by default).
- Pilot RC radio stick deadzone (0 by default).

## v0.7.1
- Second window: analog look off by default.

## v0.7.0
- Second window drawn with OpenGL on its own thread: Catmull-Rom upscale, supersampled capture
  (1.5x default), sharpening; auto exposure drives the capture's own AutoExposureBias.

## v0.6.x
- Flying wing (Mojito-like), inverted gimbal tilt down to -110 deg, option to hide the game UI,
  Inspire yaw spin fix when flying forward / backward, safe injection while the game loads,
  support for the UEVR engine tick when available.

## v0.5.x
- Second window (F10) with the other camera (pilot FPV / gimbal), analog feed option, auto
  exposure; Dual OP replay can be flown with the FPV camera; OSD, crosshair and Ronin drift off
  by default; gimbal angles / speeds from the Assetto Corsa mod.

## v0.4 and earlier
- Acro flight model, Betaflight / KISS / Actual rates per axis, collisions with bounce, optional
  crash, arm switch on the radio, pro cinema mode (Inspire-like flight, stabilized gimbal, focal
  length 6-200 mm), Dual OP live and replay.
