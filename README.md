# Automatic Page Turner

An assistive device that turns book pages hands-free. A press of a button or a spoken command triggers a sequence of servo clamps and a rubber wheel that lift a single page and flip it over.

Built as a team project for RPI's Introduction to Engineering Design (IED) course.

This is the medium where we will upload our work for the IED page turner

## How it works

1. **Clamps** hold the book open and release on command.
2. A **rubber wheel** spins to lift the top page.
3. A **flipper arm** sweeps the page over, then the clamps re-engage.
4. Input comes from a **button** or a **voice recognition module** trained on a turn command.
5. An optional **ultrasonic rangefinder** was explored for page detection.

## Hardware

- Arduino
- Servo motors (clamps and flipper arm)
- DC motor with rubber wheel
- Voice recognition module
- Push button
- Ultrasonic rangefinder (experimental)

## Files

| File | Purpose |
|---|---|
| `control_scheme.ino` | Main control sequence (button-triggered) |
| `control_scheme_addedranger.ino` | Control sequence with the rangefinder added |
| `WIP_voice_servo_clamp.ino` | Voice-triggered clamp and flip sequence |
| `wheel_control.ino` | Wheel motor control |
| `Ranger_tester_code.ino` | Rangefinder test |
| `readme` | Team handoff notes: wiring checks and voice module training steps |

## Status

Course prototype. The clamp and flipper sequence works; voice control and the rangefinder were in testing at the end of the semester.
