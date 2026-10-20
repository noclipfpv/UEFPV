# UEFPV — FPV drone plugin for UEVR

A **universal FPV drone** for Unreal Engine games on a **flat screen**, built as a native plugin
for [praydog's UEVR](https://github.com/praydog/UEVR). Drop one DLL in UEVR's plugin folder,
inject any UE4/UE5 game, press **F7** and fly an acro quad through the game world.

No per-game code: everything goes through Unreal's reflection system. No VR headset or VR
runtime is needed; UEVR is only used as the injector and plugin host.

## Features

- Acro flight model running at 500 Hz: thrust-to-weight, motor spool lag, anisotropic quadratic +
  linear drag, camera uptilt.
- Rates per axis (roll / pitch / yaw) in the three Betaflight configurator systems:
  **Betaflight** (RC Rate / Super Rate / RC Expo), **KISS** (RC Rate / Rate / RC Curve) and
  **Actual** (Center Sensitivity / Max Rate / Expo), with live curves. Formulas from Betaflight's
  `rc.c`.
- **Collisions** with the game world: sphere sweep every frame, the drone bounces and slides along
  walls and floors (bounce, friction, tumble and radius adjustable). Optional crash mode: hard
  impacts disarm the quad, it falls, and re-arms with throttle low once at rest (auto-flip if it
  landed upside down).
- Optional angle (self-level) mode.
- **Pro cinema mode** (DJI Inspire-like): velocity flight with position / altitude hold, smooth
  acceleration and braking, visual body tilt; **stabilized 3-axis gimbal** with Ronin-like inertia,
  tilt -110..+85 deg (inverted gimbal), optional Ronin roll drift under lateral g, **focal length** 6-200 mm on a full frame /
  Super 35 / Micro 4/3 sensor with zoom.
- **Second window** (F10): the other view (pilot FPV or gimbal camera) in its own window, for a
  second monitor or OBS, rendered by a scene capture at a chosen resolution / refresh rate, with an
  optional **analog feed simulation** (light grain, scanlines, soft chroma) that hides the lower
  resolution. Double-click it for fullscreen on its monitor.
- **Dual OP**: live with a second gamepad for the camera operator (pilot on the radio or gamepad 1),
  or "replay" for a single person: fly, then replay the flight (real time or slow motion, looped)
  while operating the camera with the same sticks.
- Inputs: Xbox/XInput gamepad (mode 2), or an **RC radio in USB joystick mode** (RadioMaster,
  TBS, Jumper, ... or any HID joystick) with axis mapping, inversion and calibration.
- Arming with **throttle low** (default) or with an **arm switch** on the radio (joystick button or
  axis), Betaflight-style: arming needs the throttle low, switch off cuts the motors (the quad
  falls), after a crash the switch must be cycled. Until the first arm the drone hovers in place.
- While flying, the player is frozen (move / look input ignored) and the gamepad is hidden from
  the game (both optional).
- Betaflight-style crosshair (can be turned off) and an optional telemetry OSD (off by default):
  mode, throttle, speed, altitude, vertical speed, distance. Arming / crash / pause warnings are
  always shown.
- In-game settings menu (ImGui) with a status panel, saved **per game** in UEVR's game folder.
- UE4 and UE5 (single and double precision / Large World Coordinates) detected at runtime.

## Controls

Note: while the F6 menu is open, flight (and arming) is paused unless "Pause in menu" is unchecked.

| Action | Keyboard | Gamepad |
| --- | --- | --- |
| Settings menu | F6 | — |
| Drone on / off (launches from the current game camera) | F7 | hold BACK + A |
| Respawn at the launch point (motors off) | F8 | hold BACK + B |
| Arm | lower the throttle | lower the throttle |
| Dual OP replay: start / stop | F9 | hold BACK + X |
| Dual OP replay: clear the take | hold F9 1 s | hold BACK + Y |
| Second window on / off | F10 | — |
| Gimbal zoom in / out | Page Up / Page Down | RT / LT (when not used for throttle) |
| Gimbal tilt / pan offset (pilot mode) | — | D-pad up/down, left/right |

Gamepad mode 2: left stick = throttle (Y) / yaw (X), right stick = pitch (Y) / roll (X).
Throttle can also be "upper half of the left stick" or the right trigger.

## Install (testers)

You need **UEVR 1.05** (the stable release, not a nightly) and the plugin DLL from this repository's
**Releases** page. UEVR is not included here: download it from praydog's official page.

1. **Download UEVR 1.05** from [github.com/praydog/UEVR/releases](https://github.com/praydog/UEVR/releases)
   (the 1.05 release, `UEVR.zip`) and unzip it anywhere.
2. **Download the plugin**: on this repository, *Releases* → latest version → `uefpv-vX.Y.Z.zip`.
   Unzip it: it contains `uevr_fpv_drone.dll`.
3. **Inject the game once with UEVR** (start the game, start `UEVRInjector.exe`, pick the game,
   *Inject*). This creates the folder `%APPDATA%\UnrealVRMod\<GameExeName>\`. Close the game.
4. **Copy `uevr_fpv_drone.dll`** into `%APPDATA%\UnrealVRMod\<GameExeName>\plugins\`
   (create `plugins` if it does not exist). To use it in every game at once:
   `%APPDATA%\UnrealVRMod\UEVR\plugins\`.
   If Windows blocks the file: right-click → Properties → *Unblock*.
5. Close SteamVR / Oculus (no headset needed), start the game, inject it with UEVR.
   UEVR saying VR is not available is expected.
6. In game: **F6** = menu, **F7** = drone on / off, **F8** = respawn, **F10** = second window.

**RC radio**: plug it in USB joystick (HID) mode *before* starting the game, then F6 → Input →
*RC radio / joystick*, pick it in the detected list, *Calibrate*, map the axes, **Save**. Settings
are per game (`%APPDATA%\UnrealVRMod\<GameExeName>\fpv_drone.ini`), so do it once per game.
If the radio is not listed in a game where it works elsewhere: Steam → game *Properties* →
*Controller* → *Disable Steam Input*, then restart the game.

The plugin is built for **UEVR 1.05**. Other UEVR versions may refuse it or misbehave (nightlies
flicker in flat-screen mode on some games, with or without the plugin).

### Installation (FR, testeurs)

1. Télécharge **UEVR 1.05** (version stable, pas une nightly) sur
   [github.com/praydog/UEVR/releases](https://github.com/praydog/UEVR/releases) et dézippe-le.
2. Télécharge le plugin : *Releases* de ce dépôt → dernière version → `uefpv-vX.Y.Z.zip`.
3. Injecte le jeu une première fois avec UEVR (ça crée `%APPDATA%\UnrealVRMod\<NomDuJeu>\`), puis ferme le jeu.
4. Copie `uevr_fpv_drone.dll` dans `%APPDATA%\UnrealVRMod\<NomDuJeu>\plugins\` (crée le dossier si besoin).
5. Relance le jeu et injecte avec UEVR. **F6** menu, **F7** drone, **F8** respawn, **F10** seconde fenêtre.
6. Radio : branchée en mode joystick USB avant le jeu, F6 → Input → *RC radio / joystick*, calibrer, **Save**
   (réglages propres à chaque jeu). Radio absente ? Désactive Steam Input pour ce jeu dans Steam.

**Report a bug / Signaler un bug**: open an *Issue* and attach `log.txt` and `fpv_drone.ini` from
`%APPDATA%\UnrealVRMod\<GameExeName>\`.

## Troubleshooting

Open the menu (F6) and look at **Status**, then at the UEVR log
(`%APPDATA%\UnrealVRMod\<GameExeName>\log.txt`, lines starting with `[FPV Drone]`).

| Symptom | What it means |
| --- | --- |
| No `[FPV Drone] ... initialized` line in the log | The DLL is not in a `plugins` folder UEVR reads, or it is blocked (right-click → Properties → Unblock). |
| F6 shows nothing, but the log has `initialized` | The overlay did not start; look for `overlay initialized` in the log and send the log. |
| Status "Game thread tick: not installed / FAILED" | The game-thread hook could not be installed; send the log. |
| Tick is running but "Camera: waiting for engine classes" | UEVR has not found the game's objects yet; wait for the game to finish loading. |
| Camera "active" but the view does not change | The game uses a custom camera manager that ignores view targets (see limitations). |
| "View target taken back from the game N times" climbing fast | The game keeps forcing its own camera; the drone wins each frame but you may see flicker. |

## Build (Windows)

Requirements: Visual Studio 2022 (C++ workload), CMake ≥ 3.24, Git.

```bat
cmake -S . -B build -G "Visual Studio 17 2022" -A x64
cmake --build build --config Release
```

Output: `build\Release\uevr_fpv_drone.dll`.

CMake fetches UEVR at a pinned commit **without its submodules** (only the MIT-licensed
plugin headers and the example ImGui render helpers are used), so you do **not** need access
to the EpicGames GitHub organization that a full UEVR build requires. Dear ImGui is fetched
at the same commit UEVR uses. Pushing the repository to GitHub builds the DLL automatically
(`.github/workflows/build.yml`).

The flight model has portable tests: `cmake --build build --config Release --target fpv_sim_tests`
then run `build\Release\fpv_sim_tests.exe`.

## How it works

UEVR only installs its engine-tick and viewport hooks when a VR runtime is loaded, so in
flat-screen mode the plugin brings its own:

```
 start of every frame (game thread)                          world tick              draw
┌──────────────────────────────────────────────────┐   ┌──────────────────────┐   ┌───────────┐
│ PumpMessages ─▶ WH_GETMESSAGE hook (our tick msg) │   │ PlayerCameraManager   │   │ renders   │
│   poll sticks, DroneSim.update(dt)                │──▶│ reads the view target │──▶│ from the  │
│   move our CameraActor (view target)              │   │ = our CameraActor     │   │ drone     │
└──────────────────────────────────────────────────┘   └──────────────────────┘   └───────────┘
 render thread: on_present ─▶ ImGui menu + OSD, installs the hook on first frame
```

- **Game-thread tick** (`GameThreadTicker`): Unreal pumps the game window's message queue on the
  game thread at the start of each frame. A `WH_GETMESSAGE` hook on that thread, plus one private
  message always kept waiting in the queue, gives a callback every frame, before the world ticks.
- **Camera** (`GameCamera`): an `ACameraActor` is spawned with
  `GameplayStatics::BeginDeferredActorSpawnFromClass` / `FinishSpawningActor` and set as the view
  target with `PlayerController::SetViewTargetWithBlend`. Each tick it is moved with
  `K2_SetActorLocationAndRotation`; its `CameraComponent.FieldOfView` sets the FOV. Turning the
  drone off gives the view back to the pawn and destroys the actor. After a level change the
  camera is re-spawned automatically.
- **Reflection calls** (`Reflection`): parameter buffers are built from each UFunction's own
  reflection data (offsets, sizes, float vs double vectors), so the same code runs on UE4 and UE5.
- **Collisions**: each tick, `KismetSystemLibrary::SphereTraceSingle` sweeps the drone's sphere from
  its previous to its new position (the camera actor is the world context and is ignored). On a hit
  the sim gets the contact point and normal (`DroneSim::apply_contact`) and handles bounce, friction,
  tumble and optional crash; the rest of the movement slides along the surface (up to 3 contacts per
  frame).
- **Flight model** (`DroneSim`): engine-independent, SI units, Unreal axes (X forward, Y right,
  Z up), position relative to the launch point; "Units per meter" converts to the game's scale.

## Cinema / Dual OP

F6 → *Cinema / Dual OP*:

- **Flight mode** — *FPV (acro)* or *Cinema (Inspire-like)*. In Cinema, the right stick (pitch /
  roll) commands forward / sideways speed, the throttle stick centered holds altitude (up = climb,
  down = descend), yaw turns. Let go of the sticks and the drone brakes and holds its position.
  Arming is unchanged (throttle low or arm switch), then center the throttle to hover.
- **Camera** — *FPV camera* (fixed to the frame, uptilt) or *Gimbal (stabilized)*. Both work with
  both flight modes.
- **Camera operator** (gimbal):
  - *Pilot*: the gimbal follows the heading smoothly; tilt with the D-pad or a pot on the radio.
  - *Dual OP - second gamepad*: operator on another XInput pad (right stick pan / tilt, left stick X
    roll, RT / LT zoom).
  - *Dual OP - replay*: every flight is recorded; F9 replays it (speed x0.1 to x1, looped) and the
    sticks operate the camera (roll = pan, pitch = tilt, yaw = roll). Hold F9 to clear the take.
- **Lens** — focal length (live), sensor size, zoom speed. **Gimbal** — Ronin roll drift strength
  and recovery, pan / tilt / roll speeds, smoothing, tilt limits.

## Flying wing (v0.6)

F6 → Flight mode → *Flying wing (Mojito-like)*. Same sticks and rates as FPV acro (scaled per axis
in *Wing setup*). Arm as usual, then raise the throttle: hand launch. The model is a flight
controller in rate mode on an elevon wing: control authority drops at low airspeed, the airframe
weathervanes into the airflow (centered sticks = it glides along its flight path, ~8:1), throttle
mostly makes it climb, and past the stall angle lift breaks down. FPV camera only (no gimbal or
Dual OP). OSD shows the angle of attack and STALL.

## Second window

F6 → *Second window (F10)*. The main screen keeps its full quality (DLSS / TAA, full post-process);
the window shows the other camera: the pilot's FPV feed when the main screen is the gimbal camera,
or the gimbal camera. It is an extra scene render, so keep it cheap: 480p 4:3, clean feed by default (analog look optional), refresh every 2nd frame if you need more FPS. Scene
captures skip the upscaler / temporal AA, so the image is a bit rawer than the main view (the
analog look hides it). *Auto exposure* (on by default) brightens the feed toward a normal level, since scene captures often
miss the game's eye adaptation; fine-tune with *Exposure (EV)* and *Gamma*. *Shows: Auto* puts on
the second window whichever view is not on the main screen.

How it works: an `ASceneCapture2D` renders into a `UTextureRenderTarget2D` (optionally supersampled
1.5x / 2x); its GPU texture is located behind the UObject and UEVR returns the native D3D11 / D3D12
resource. From UEVR's present callback the capture is copied into CPU-readable memory on the game's
own device (never waiting on the GPU). The window's own thread draws it with OpenGL (no DXGI, so
nothing runs inside UEVR's / ReShade's swapchain hooks): box downscale of the supersampled capture
or Catmull-Rom upscale, contrast-limited sharpening, then the optional analog look. Auto exposure
drives the capture's own `AutoExposureBias` (HDR, before tone mapping: no banding). GDI fallback if
OpenGL is unavailable.

## RC radio setup

Arm switch: F6 → Input → Arming → "Arm switch". Flip the switch and watch "Buttons pressed now" or
the axis bars to find which one it is, pick it, set the threshold for an axis, and check that
"Arm switch" shows ON/OFF correctly. Windows' legacy joystick API sees 6 axes and 32 buttons; if
the switch channel is not visible, map it to a button in the radio's USB joystick settings
(EdgeTX: Model → USB joystick).


1. Put the radio in USB joystick (HID) mode and plug it in.
2. F6 → Input → "RC radio / joystick". Pick the joystick id whose bars move.
3. Click **Calibrate**, move every stick to all corners, click **Finish calibration**.
4. Map Throttle / Roll / Pitch / Yaw to the right axes and invert where needed
   (watch the channel bars at the bottom of the Input section). **Save**.

## Hide the game's UI

F6 → Camera → *Hide the game's UI while flying*. Hides the game's UMG widgets that are on the
viewport (and the legacy AHUD) while the drone is active, rechecks twice a second for new widgets,
and restores each one to its previous visibility when you leave the drone. Off by default.

## Dual OP live: operator on a gamepad or an RC radio

F6 → Cinema → Camera operator: *Dual OP - second operator (live)* → *Operator input*:
- **Gamepad** (default): right stick pan / tilt, left stick X roll, RT / LT zoom. Own deadzone.
- **RC radio / joystick**: pick the operator radio's joystick id, map Pan / Tilt / Roll / Zoom to its
  axes (mode 2 defaults: right stick pan / tilt, left stick X roll, throttle stick = focal length),
  calibrate it, optional deadzone (0 by default). Zoom can follow the stick / pot (focal length) or
  act as a zoom speed on a centered stick.

The pilot's radio has its own stick deadzone too (Input → RC radio), 0 by default.

## Known limitations (v0.7.7)

- The plugin does not touch the game's objects until you press F7 the first time, so it is safe to
  inject while the game is still starting. Press F7 once the game is loaded (menu or in game).

- Collisions use the game's collision on the chosen trace channel (Visibility by default, Camera as
  an option). Objects without collision on that channel (some foliage, effects) are flown through.
- Starting a sweep inside geometry (e.g. spawning inside the player's capsule) is ignored so the
  drone can fly out.
- The player character stays where it was and remains visible.
- Level streaming and LODs stay centered on the player pawn, so very long flights may reach
  unloaded areas.
- Games whose custom `PlayerCameraManager` ignores the view target (fully scripted cameras) are
  not covered yet.
- The game's camera shakes / post-process from the camera manager may still apply.
- Flat screen only for now. VR support may come back later.

## Roadmap ideas

Turtle mode, hiding the player pawn (optional), saving takes to disk, recording the gimbal
moves of a take, depth of field / focus pulling, propwash.

## Credits / license

- UEVR and its plugin API by praydog. The API headers (`API.h`, `API.hpp`, `Plugin.hpp`) are
  MIT-licensed; UEVR's example render helpers are compiled into the DLL as intended for plugin
  authors. No UEVR source is redistributed in this repository — CMake fetches it.
- Dear ImGui by Omar Cornut (MIT).
- This plugin's own code: MIT (see `LICENSE`).
