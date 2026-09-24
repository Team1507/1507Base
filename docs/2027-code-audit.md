# 1507Base 2027 Code Audit

**Branch:** `SystemCore` (WPILib 2027 alpha 7, Phoenix 6 `26.70.0-alpha-2`, Commands v3)
**Audit date:** 2026-09-24 (second full audit, after the first audit's fixes and the redesign work)
**Scope:** every file in `src/`, the subsystem template, build files, vendordeps, CI, and the wiki pages the code points students to. About 5,950 lines of main code and 1,050 lines of tests.
**Build:** `./gradlew build` passes locally and on GitHub Actions. **34 unit tests, 0 failures.**

> The first audit (the findings that led to this session's work) is in this file's git history: `git log -p docs/2027-code-audit.md`.

This audit checks the code against four goals:

1. **Easy for students.** Going from skeleton to working robot code, including changes at an event, should take as little code and as few "remember to do this" steps as possible. Complexity belongs in `lib/`, not `robot/`.
2. **No loop overruns.** Nothing that runs every 20 ms should do more work than it needs to.
3. **Logs everything we want.** After a match, the log should answer: what was the battery doing, who was drawing current, and what commands were running?
4. **Comments are accurate.** This is the skeleton project, so comments explain *how and why*, and they must match the code.

| Rank | Meaning |
|---|---|
| 🔴 Critical | Wrong behavior on the real robot. |
| 🟠 High | Works, but breaks a goal above in a way students or matches will hit. |
| 🟡 Medium | Will cost time later, or needs a decision. |
| ⚪ Low | Cleanup or polish. |

---

## Completed this session

| Area | What changed |
|---|---|
| **First-audit critical fixes** | Drive kV units (C1), Pigeon yaw at 4 Hz (C2), CANcoder offset frames (C3), current limits (H1), config retry (H3), odometry jump from the wrapping CANcoder |
| **2027 hardware** | SDS MK5n, Kraken X60/X44, 4" wheels, Phoenix Pro (FusedCANcoder, FOC) |
| **Tuner X paste zone** | Swerve settings pasted straight from Tuner X; `SwerveConfig.java` holds the robot's drivetrain hardware, `Swerve.java` holds how it drives |
| **Heading (H4)** | Driving is driver-relative on both alliances; the zero button sets the field heading correctly on Red |
| **Commands v3** | Replaced v2 and `CommandBuilder`; OpMode-scoped bindings (fixed M3); auto timer starts automatically (fixed M2b) |
| **MotorConfig** | Presets (`roller`, `flywheel`, `arm`, `elevator`, `position`), `copy()`, `withCtreConfig`, supply lower limit, gear ratio, drum diameter, tolerances, `problems()` |
| **Motor1507** | Commands in degrees/RPM/meters/inches, `isAtTarget()`, `resetPosition()`, `get(MotorSignal)` for every signal, `CommonTalon`, reused control requests |
| **Swerve cleanup** | Removed `driveRobotRelative` (M2); shared helpers (`driveToTarget`, `turnToFace`, `ProgressWatchdog`); motor configs built once and copied |
| **Overrun fix** | `Telemetry.set()` caches NetworkTables publishers (the 2026 fix that never reached `main`; later replaced by WPILib `Telemetry`, which caches the same way) |
| **Cleanup** | AdvantageKit removed (M6); unused imports; the gear-ratio check the IDE flagged as dead code |
| **Build-time checks** | `SwerveConfigTest`, `MotorConfigConstantsTest`, `MotorConfigTest`, `Motor1507Test`, `SwerveCommandsTest`, `CommandsV3PatternsTest` (+ `NodeBoundsTest`); GitHub Actions runs them on every push |
| **Docs** | Season Setup Checklist with matching `TODO(SEASON SWERVE-n)` tags; wiki rewritten for 2027 (Home, Getting Started, Swerve Drive, Motor Configuration, Adding a Subsystem, Button Bindings, Autonomous, Code Architecture) |
| **Step 3: `Subsystem1507`** (fixed H-1) | `motor(...)` creates and registers a motor; every loop the base class refreshes all its motors in one CAN call, runs `periodic()`, logs `<Subsystem>/TotalSupplyCurrent`, and steps the sim. Default CAN bus set once in `Robot.java`. `warnIf`/`faultIf` alerts clear themselves. Swerve's 8 motors count toward its current. `Subsystem1507Test` |
| **Motion Magic** (fixed M-1) | `withMotionMagicDegrees` / `withMotionMagicMeters` / `withMotionMagicRotations`; the build fails if Motion Magic has no speed limits |
| **Logging work** (fixed H-2, M-4) | Moved onto WPILib 2027 `Telemetry`. Every motor logs all 15 signals + target + stall every loop (motor supply voltage raised to 50 Hz, matching the currents); `RobotHealthLog` logs battery, brownout, loop period/time, CAN utilization per port, PDH channels, total motor current; `CommandLog` logs every command start/end (`Commands/Events`) and each subsystem's current command; each subsystem logs its `PeriodicMs`. Our `Telemetry`, `InputField` and `TelemetryRate` deleted. Also: the Pigeon's startup read no longer prints a CAN error. Telemetry and Logging wiki page rewritten; LOGGING-1..3 added to the checklist |
| **Comments (this audit)** | 13 stale or wrong comments fixed; see [Goal 4](#goal-4-comment-accuracy) |

---

## Summary

The code is in much better shape than at the first audit. **No critical findings remain in code.** The open critical work is on the robot: verifying swerve on blocks and running SysId. The drivetrain is correct on paper, heavily tested, and set up for Tuner X.

Both High findings are fixed: student subsystems no longer need boilerplate (H-1, Step 3), and the log now records everything needed to diagnose a brownout (H-2, the logging work). **No High findings remain.** What's left is Medium: measuring CAN load and loop time on the real robot, sim fidelity, and decisions.

Loop efficiency is good: nothing heavy runs every loop, and loop time is now logged (see [Goal 2](#goal-2-no-loop-overruns)).

---

## Open findings

### 🟠 High

#### ~~H-1. Student subsystems must remember `refresh()` and `simulationPeriodic()`~~ ✅ Fixed (Step 3)
Motors created with `Subsystem1507.motor(...)` are refreshed (one CAN call per subsystem) before `periodic()` and stepped in sim automatically. `Subsystem1507Test` checks it with a subsystem that has no refresh or sim code.

_No open High findings._ H-1 and H-2 are fixed; see **Completed**.

### 🟡 Medium

#### ~~M-1. `ControlMode.MOTION_MAGIC` silently doesn't move~~ ✅ Fixed
`MotorConfig` now has `withMotionMagicDegrees` / `withMotionMagicMeters` / `withMotionMagicRotations`, and `problems()` fails the build if Motion Magic has no cruise velocity or acceleration.

#### M-2. CAN bus load is unmeasured on the real robot
Each `Motor1507` now reports 15 signals (position/velocity at 100 Hz, current/voltage at 50 Hz, the rest at 10 Hz). The swerve alone is 8 motors, 4 CANcoders and a Pigeon 2 on one SystemCore CAN port, which runs classic CAN (not CAN FD). This could get tight once mechanisms are added to the same port.
**Partly fixed:** `Robot/CAN/<port>/Utilization` is now logged. **Remaining:** read it on the real robot (checklist LOGGING-3); if a port runs high, put mechanisms on a different port from the drivetrain, or slow unneeded signals.

#### M-3. Simulation doesn't run the real control path
`SwerveModule1507` keeps its own fake drive/steer model, and `Motor1507` has a simple "move toward target" model. Neither exercises CTRE's control loop, gains or current limits, so sim can't catch tuning or current-limit mistakes (the original M1).
**Fix:** Phase 3, CTRE sim state + WPILib `DCMotorSim` + a simulated battery.

#### ~~M-4. `InputField` publishing creates garbage every loop~~ ✅ Fixed
`InputField` is deleted. Everything logs through WPILib `Telemetry` with primitive values.

#### M-5. Adding a subsystem to autos needs `AutoBuilder` registration
A subsystem used in autos must be added to `AutoBuilder` (field + `init()` parameter + the `Robot.java` call). Forgetting causes a `NullPointerException` at enable. **On hold by choice**: autos will be revisited once the other systems are in place.

#### M-6. Decisions needed
- **CAN port layout** (was M4): which devices go on which SystemCore port.
- **Vision trust** (was M5): `VISION_STD_DEV` equals `ODOMETRY_STD_DEV`. Revisit when QuestNav (with AprilTags) ships for 2027, since it will be the main source of field position.

### ⚪ Low

- **Wiki page still describing 2026:** QuestNav (flagged on the Home page).
- **Season Setup Checklist** covers Swerve and Logging. Field/auto, vision and project setup sections are still to be written.
- **Sub-loop current spikes:** motor signals are logged once per loop (50 Hz). CTRE's `.hoot` log would catch shorter spikes; confirm it runs on SystemCore before relying on it.
- `RobotMap.OPERATOR_CONTROLLER` is defined but unused (fine for a skeleton; the operator gamepad isn't created yet).
- **Cosine scaling** for swerve modules (slow the wheel while it's still turning) is planned for Phase 2.
- `roller()` (torque control) on a **TalonFXS/Minion** is untested. TalonFX is confirmed.

---

## Goal 1: Easy for students

What a student writes to add a mechanism today:

| Step | File | Effort | Notes |
|---|---|---|---|
| CAN ID | `Constants.RobotMap` | 1 line | |
| Motor config | `Constants.kX` | 3–5 lines with a preset | ✅ Checked at build time |
| Subsystem class | `robot/subsystems/X.java` | Template generates it | ✅ `motor(...)` handles refresh, sim and current logging (Step 3) |
| Commands | subsystem | `runRepeatedly(...)`/`run(...)` + `.named()` | ✅ Goal-based motor calls in degrees/RPM |
| Default command | `Robot.java` | 1 line | |
| Button bindings | `DriverTeleop` | 1 line each | ✅ Removed automatically with the OpMode |
| Use in autos | `AutoBuilder` + `AutoSequence` | 3 places | ⚠️ M-5 (on hold) |

**Strong points:** presets hide CTRE's config names; motor calls take the units students think in; the build catches bad configs, swerve pastes and field positions; the wiki examples compile against the real code.

**Remaining friction:** M-5 (autos, on hold).

---

## Goal 2: No loop overruns

Everything that runs every 20 ms, and its cost:

| Every loop | Cost | Verdict |
|---|---|---|
| Scheduler, triggers, running commands | Small | ✅ |
| Swerve `refreshAll` (one non-blocking CAN call for ~130 signals) | Small | ✅ |
| Pose estimator update, kinematics | Small | ✅ |
| Motor logging: 18 values × every motor (~290 calls with swerve + 8 mechanism motors) | Measured on a laptop: ~0.03 ms for 16 motors; the whole sim loop (`Robot/RobotPeriodicMs`) ~0.55 ms, Swerve ~0.26 ms. SystemCore is slower, and real values change every loop | ✅ Check `Robot/RobotPeriodicMs` on the robot (LOGGING-3) |
| Other `log()` calls (swerve: ~8/loop), robot health (~8/loop, CAN at 10 Hz) | Small; WPILib `Telemetry` caches entries per table | ✅ |
| Command events | Only on start/end; one set lookup per running command per loop | ✅ |
| Stall checks | Tiny | ✅ |
| Alliance lookup in teleop driving | Tiny | ✅ |

**Nothing blocks, and nothing does per-loop string lookups anymore.** The only overrun seen in sim is the **first loop** after startup (about 0.12 s), which is one-time initialization, not a recurring cost.

What to watch as the season code grows:
- **Blocking CAN calls in `periodic()`**, such as `getConfigurator().apply(...)` or `setPosition(...)` every loop. The 2026 Hopper avoided this by zeroing only on the limit switch's rising edge; keep that pattern.
- **Anything created every loop that could be created once** (control requests, publishers, arrays). The base classes already do this.

Loop timing is now in every log: `Robot/LoopPeriodMs` (20 = on time), `Robot/RobotPeriodicMs`, and `<Subsystem>/PeriodicMs` to find the slow one.

---

## Goal 3: Logs everything we want

| Brownout question | Answered by | Status |
|---|---|---|
| How low did the battery go, and when? | `Robot/BatteryVoltage`, `Robot/BrownedOut`, each motor's `SupplyVoltage` | ✅ |
| Which mechanism drew the current? | `<Subsystem>/TotalSupplyCurrent`, each motor's `SupplyCurrent`, `PDH/Current` per channel | ✅ |
| What was the robot doing? | `Commands/Events`, `<Subsystem>/Command`, Driver Station inputs | ✅ |
| Was a mechanism stalled or faulted? | Each motor's `Stalled`, `StatorCurrent`, `Faults`, `TemperatureC` | ✅ |
| Was the code or the CAN bus struggling? | `Robot/LoopPeriodMs`, `<Subsystem>/PeriodicMs`, `Robot/CAN/<port>/Utilization` | ✅ |

Everything goes to one `.wpilog` (USB drive if present), opens in AdvantageScope, and exports to CSV for review by Claude or a spreadsheet. The [Telemetry and Logging](https://github.com/Team1507/1507Base/wiki/Telemetry-and-Logging) wiki page walks through a brownout investigation.

**Still open (Low):** motor signals are sampled once per loop (50 Hz). A spike shorter than 20 ms is still missed; CTRE's `.hoot` log records signals at their own rates if that's ever needed (confirm it runs on SystemCore).

---

## Goal 4: Comment accuracy

Every comment was read against the current code. These were stale or wrong, and **were fixed in this audit** (no code behavior changed):

| File | Was | Now |
|---|---|---|
| `Constants` | "all CAN IDs in one place" | Drivetrain IDs live in `SwerveConfig` |
| `Constants` | stall settings "for moveThroughPose" | Used by all three auto driving commands |
| `AutoBuilder` | "exposes zero-argument command factories", step 4 "add your command factory methods" | Holds references only; commands live on subsystems |
| `AutoSequence` | "Add your command to AutoBuilder.java" | Write it on the subsystem; register the subsystem |
| `AutoSequence` | `slow()`/`creep()` "75%/50% angular rate" | 0.75 / 0.5 rotations per second (what the code does) |
| `AutoSequence` | `withDebug` example calls `.startTimer()` | Not needed; the timer starts automatically |
| `AutoSequence`, `Subsystem1507` | names "show up in command logs" | Said "arrive with the logging work"; now true (`Commands/Events`) |
| `Subsystem1507` | motors publish at `Arm/MotorA/Input/Position` | `Arm/MotorA/PositionDeg`, `Arm/MotorA/RPM` |
| `Telemetry` | handles "command lifecycle" logging | Removed with `CommandBuilder`; notes the planned replacement |
| `Telemetry`, `VisionConsumer`, `Swerve` | "FPGA timestamp" | Robot timestamp (`Timer.getTimestamp()`); SystemCore has no FPGA |
| `CtreMotorConfigurator` | CANcoders configured in `SwerveModule1507` | In `SwerveConfig` |
| Subsystem template | no mention of `refresh()` or sim | Explains both (H-1); since Step 3, `motor(...)` does both and the template says so |
| Wiki: Adding a Subsystem | example had no sim step | Added `simulationPeriodic()`; since Step 3, uses `motor(...)` and needs neither |

Comments added during this session (Motor1507, MotorConfig, SwerveConfig, Swerve, the v3 framework classes) were re-read and match the code.

---

## What WPILib 2027 recommends vs. what we do

| Area | 1507Base now | Status |
|---|---|---|
| Robot structure | `OpModeRobot` + `@Autonomous` / `@Teleop` | ✅ Matches |
| Commands | Commands v3 | ✅ Matches the new framework (alpha; `CommandsV3PatternsTest` guards our patterns) |
| Dashboard inputs | `Tunables` | ✅ Matches |
| Log to disk | `DataLogManager` + `DriverStation.startDataLog` | ✅ Matches |
| Telemetry | WPILib `Telemetry` (NT backend under `/Telemetry`, recorded by `DataLogManager`) | ✅ Matches |
| Power / CAN monitoring | `RobotHealthLog`: battery, brownout, PDH, CAN utilization, loop time | ✅ |
| Command logging | Commands v3 `addEventListener` → `CommandLog` | ✅ Matches the recommended v3 mechanism |
| CTRE config | Retry + DS error; offsets on the CANcoder; explicit signal rates | ✅ Matches CTRE guidance |
| CTRE high-rate data | `.hoot` starts automatically | ⚠️ Confirm on SystemCore; only needed for sub-20 ms spikes |

---

## Recommended next steps

1. ~~**Step 3: `Subsystem1507`**~~ ✅ Done (fixed H-1, and M-1 rode along).
2. ~~**Logging work**~~ ✅ Done (fixed H-2 and M-4; M-2 now measurable).
3. **Robot time** (when the MK5n modules are built) — Season Setup Checklist SWERVE-3 to SWERVE-8, and LOGGING-1 to LOGGING-3.
4. **Phase 3 simulation** (M-3), then **Phase 2 swerve library** (odometry thread, current budget, acceleration limiting, cosine scaling).
5. **Revisit autos** (M-5) and **restore QuestNav** once its 2027 build ships.

---

## Reference: swerve values vs. Team 340 (2026)

340 runs the same MK5n + Kraken X60 (FOC) combination, so their values are our starting point. Units match exactly: drive gains in volts per **motor** rps, steer gains in volts per **module** rotation.

| Setting | 1507 (now) | 340 | Status |
|---|---|---|---|
| Drive kV / kP | 0.125 / 0.25 | 0.125 / 0.25 | Matches; confirm with SysId (SWERVE-7) |
| Steer kP / kD | 100 / 0.2 | 100 / 0.2 | Matches |
| Drive stator / supply | 80 A / 28 A | 80 A / 28 A | Matches; measure slip current (SWERVE-8) |
| Steer stator / supply | 60 A / 40 A | 60 A / 40 A | Matches |
| Steer ratio | 287/11 | 287/11 | Matches (MK5n) |
| Drive ratio | 6.12 (2026 ratio) | 6.03 | Confirm at assembly (SWERVE-4) |
| Coupling ratio | 3.57 (MK4i value) | — | Take from Tuner X (SWERVE-4) |
| Encoder offset | Written to the CANcoder | Written to the CANcoder | Matches |
| Acceleration limiting | None | Slip + torque limits | Phase 2 |
| Odometry | 50 Hz in `periodic()` | 50 Hz odometry thread | Phase 2 decision |
