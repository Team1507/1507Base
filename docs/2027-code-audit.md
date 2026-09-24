# 1507Base 2027 Code Audit

**Branch:** `SystemCore` (WPILib 2027 alpha 7, Phoenix 6 `26.70.0-alpha-2`)
**Date:** 2026-09-24
**Scope:** every file in `src/`, the build files, vendordeps, the subsystem template, and the parked QuestNav code. About 5,600 lines.
**Build status:** `./gradlew build` passes, including `NodeBoundsTest`.

This audit answers three questions:

1. **What does the code do today?**
2. **What do WPILib/FIRST and our vendors recommend for 2027?**
3. **What has to change before this becomes the base code for every SystemCore season?** The focus is swerve and data logging, because brownouts cost us matches in 2026.

Findings are ranked:

| Rank | Meaning |
|---|---|
| 🔴 Critical | Wrong behavior on the real robot. Fix before the robot drives on this code. |
| 🟠 High | Works, but hurts reliability or hides the data we need to diagnose problems. |
| 🟡 Medium | Design problem that will cost us time later in the season. |
| ⚪ Low | Cleanup or polish. |

---

## Summary

The 2027 migration itself is solid. The OpMode conversion, the immutable-math fixes (`optimize()`, `desaturateWheelVelocities()`) and the CAN bus changes are all done correctly. The rename is complete, and the migration doc is excellent.

The serious problems were **not** caused by the migration. They are 2026 code that came across unchanged, and some of them match the brownout symptoms we saw last season:

- 🔴 **The drive motors are asked for about 20× too much voltage**, so they run at nearly full power for almost any drive command (finding C1). That means maximum current draw all the time, which is a textbook brownout cause.
- 🔴 **The gyro only updates 4 times a second** on the real robot (C2).
- 🔴 **The CANcoder offset is used in two different ways**, one in software and one in the motor controller, so either steering or odometry is wrong on the real robot (C3).
- 🟠 **Current limits allow up to ~560 A of supply current** across the eight swerve motors (H1).
- 🟠 **Logging records current at only 10 Hz and battery voltage not at all.** A brownout happens in tens of milliseconds, so the logs we have now can't catch one (H2).

None of these show up in simulation. Swerve simulation skips the CTRE signal and control path entirely (M1), which is why they went unnoticed.

> **Status (2026-09-24):** C1, C2, C3, H1 and H3 are fixed in code as part of the MK5n update. Swerve hardware settings now live in a Tuner X paste zone at the top of `subsystems/Swerve.java`. That update also fixed an odometry jump: the coupling math used the CANcoder's wrapping absolute position. What's still open: verify on the robot (wheel alignment, drive direction), run SysId, and confirm the MK5n drive ratio, coupling ratio and wheel size at assembly (search `TODO(MK5n`).

---

## 🔴 Critical

### C1. Drive feedforward is in the wrong units, so the motors saturate at 12 V

[Constants.java:131-136](../src/main/java/org/team1507/robot/Constants.java#L131-L136): every drive motor uses `.withFeedforward(0.12, 2.75, 0.32)`, which means kS = 0.12, **kV = 2.75**, kA = 0.32.

Phoenix 6 `VelocityVoltage` gains are in **volts per motor rotation per second**, because the drive motor has `SensorToMechanismRatio = 1` and [SwerveModule1507](../src/main/java/org/team1507/lib/core/swerve/SwerveModule1507.java#L127) sends it motor rps. Now do the math:

- 1 m/s = 1 / (2π × 0.049581 m) = 3.21 wheel rps × 6.12 gear ratio = **19.6 motor rps**
- Feedforward at 1 m/s = 2.75 × 19.6 = **54 V**. The motor can only get 12 V.
- A realistic kV is about 12 V ÷ ~100 rps free speed ≈ **0.12 V/rps**.

2.75 looks like a value in volts per **meter per second** (for example from SysId run in meters) that was typed into a field that expects motor rps.

**What this does on the robot:** any command above about 0.25 m/s asks for more than 12 V, so the motor gets full battery voltage. kP = 2.0 pulls it back somewhat, but the robot still ends up well above the commanded speed, and it pulls maximum acceleration current every time the driver touches the stick. Across four drive motors, that is exactly how a robot browns out.

**Fix:** re-run SysId (or use CTRE Tuner X's swerve generator values) in motor-rps units. Start near kS ≈ 0.1–0.2, kV ≈ 0.12, kP ≈ 0.1. Add a unit comment on every gain in `Constants`.

> ✅ Verify first: check `git show main:src/main/java/org/team1507/robot/Constants.java`. The 2026 values are identical. If the 2026 competition robot ran this same module code, this is a strong suspect for the brownouts.

> ✅ **Cross-checked against Team 340's 2026 code** (see [Appendix A](#appendix-a-comparison-with-team-340-2026)). 340 uses the same units (motor rps, `SensorToMechanismRatio` = 1) and runs **kV = 0.125**. The motor's own numbers agree: our `SPEED_AT_12_VOLTS` of 5.04 m/s is 99 motor rps, and 12 V ÷ 99 rps = **0.121**. Our 2.75 is 22× too high.

### C2. The Pigeon2 yaw updates at only 4 Hz on the real robot

[Swerve.java:162](../src/main/java/org/team1507/robot/subsystems/Swerve.java#L162) calls `ParentDevice.optimizeBusUtilizationForAll(4.0, ..., pigeon)`. The [CTRE docs](https://api.ctr-electronics.com/phoenix6/stable/java/com/ctre/phoenix6/hardware/ParentDevice.html) say this slows down *every status signal that has not been given an update frequency with `setUpdateFrequency()`*, and the first argument is the rate it slows them to.

Motor and CANcoder signals get explicit frequencies ([CtreMotorSignals.java:69-71](../src/main/java/org/team1507/lib/core/impl/ctre/CtreMotorSignals.java#L69-L71), [SwerveModule1507.java:83](../src/main/java/org/team1507/lib/core/swerve/SwerveModule1507.java#L83)). **The Pigeon yaw never does** ([Swerve.java:145](../src/main/java/org/team1507/robot/subsystems/Swerve.java#L145)), so it drops to 4 Hz.

**Effect:** field-relative driving and the pose estimator use a heading that can be up to 250 ms old. Rotating while translating makes the robot drift, and odometry picks up error every time the robot turns.

**Fix:** `BaseStatusSignal.setUpdateFrequencyForAll(100, yaw, pigeon.getAngularVelocityZWorld())` before the optimize call. It's also worth using `BaseStatusSignal.getLatencyCompensatedValue(yaw, yawRate)`.

### C3. The CANcoder offset is in two different frames

The encoder offsets in `Constants.RobotMap` are only subtracted **in software**, in [SwerveModule1507.getAngle()](../src/main/java/org/team1507/lib/core/swerve/SwerveModule1507.java#L209-L215). But the steer TalonFX closes its position loop on the **raw CANcoder** (`FeedbackSensorSource = RemoteCANcoder`), and nothing in the code configures the CANcoder's `MagnetOffset`. (A search for `MagnetOffset` finds nothing, and no CANcoder is ever passed to a configurator.)

So `setDesiredState()` computes `targetAngle` in the offset-corrected frame and sends it to a controller that measures in the raw frame ([SwerveModule1507.java:132](../src/main/java/org/team1507/lib/core/swerve/SwerveModule1507.java#L132)). What actually happens depends on what is saved on each CANcoder from Tuner X:

| CANcoder MagnetOffset saved in Tuner X | Result |
|---|---|
| 0 | Wheels steer to the wrong angle: off by 46°, −103°, −157° and 50°. The robot would barely drive. |
| Same as `Constants` | Steering is correct, but `getAngle()` subtracts the offset **again**, so odometry module angles are wrong and `optimize()` makes bad flip decisions. |

**Fix (this is what CTRE recommends):** apply `MagnetSensorConfigs.MagnetOffset` from `Constants` to each CANcoder **in code** at startup, and delete the software subtraction. Then there's one frame, and it survives swapping a CANcoder. When it's fixed, check it on the robot: point all wheels forward and confirm `Swerve/*/Steer` angles read ~0 in AdvantageScope.

### C4. Simulation cannot catch C1–C3

Listed here because it's the reason the three bugs above survived. See M1.

---

## 🟠 High

### H1. Current limits allow ~560 A of supply current from the swerve alone

Default limits in [MotorConfig.java:148-149](../src/main/java/org/team1507/lib/core/util/MotorConfig.java#L148-L149) are 120 A stator / **70 A supply per motor**, with no lower limit and no time window. The drive configs don't override them, and steer only lowers stator to 60 A. Eight motors × 70 A supply = **560 A** possible. A battery sags below the brownout threshold well before that.

**Recommended:**
- Drive: stator ≈ 80–100 A (set it just below wheel slip, which you can measure), supply ≈ 60 A with `SupplyCurrentLowerLimit` ≈ 40 A after `SupplyCurrentLowerTime` ≈ 1 s.
- Steer: supply ≈ 20–30 A. Steering needs very little torque.
- Add a **current budget** to our swerve (see Phase 2 of the plan): when battery voltage drops, reduce drive current limits before the controller browns out.

### H2. Logging can't see a brownout

What we log today, and how often:

| Signal | Rate in our logs | Needed for brownout diagnosis |
|---|---|---|
| Battery voltage | **not logged** | ≥ 50 Hz |
| Brownout flag (`RobotController.isBrownedOut()`) | **not logged** | every loop |
| PDH per-channel current | **not logged** | ≥ 50 Hz |
| Motor supply current | 10 Hz (`NORMAL`) | ≥ 50 Hz |
| Motor stator current | 10 Hz | ≥ 50 Hz |
| Motor voltage / temperature | 2 Hz (`SLOW`) | 10 Hz is OK |
| CAN bus utilization / errors | **not logged** | 10 Hz |
| Loop time / overruns | not logged by us | every loop |

Our `Telemetry` only publishes to NetworkTables. `DataLogManager.start()` records NT to the log file, so data does reach the `.wpilog`, but only at the rates above. At 10 Hz, a 30 ms current spike happens *between* samples.

Also in the logging code:
- `Telemetry.set(...)` ([Telemetry.java:100-134](../src/main/java/org/team1507/lib/core/logging/Telemetry.java#L100-L134)) calls `getEntry(key)` on every write. That's a string lookup each time, with untyped entries.
- `InputField<Double>` boxes a new `Double` for every value it publishes, which creates garbage-collection pressure on the robot.
- **Name collision:** our `org.team1507.lib.core.logging.Telemetry` has the same name as WPILib 2027's new `org.wpilib.telemetry.Telemetry`. Students will import the wrong one.

The fix is Phase 1 of the plan below.

### H3. Config failures only print to the console

[CtreMotorConfigurator.java:393-397](../src/main/java/org/team1507/lib/core/impl/ctre/CtreMotorConfigurator.java#L393-L397): if `apply()` fails (common right after power-on while devices boot), we print one line to stdout and keep going with factory defaults. That means no current limits, the wrong inversion, and the wrong feedback sensor.

**Fix:** retry up to about 5 times, report failures to the Driver Station with `DriverStationErrors.reportError`, and log a `Config/<device>/OK` boolean.

### H4. Zeroing the heading corrupts the field pose on Red

[Swerve.zeroHeading()](../src/main/java/org/team1507/robot/subsystems/Swerve.java#L364-L373) sets the gyro to 0 **and** resets the pose estimator's rotation to 0. On Red, a robot facing away from its driver is at 180° in field coordinates, not 0°. After a zero, auto commands, `pointToTarget` and vision fusion all run on a heading that's 180° off.

**Fix:** keep two separate ideas:
- **Field heading** (pose estimator): always Blue-origin and never zeroed by the driver. It's reset by auto `resetPose` or vision.
- **Driver perspective** (teleop only): a stored offset (0° for Blue, 180° for Red) that the driver button adjusts.

This is how CTRE's swerve API handles it (`setOperatorPerspectiveForward`).

---

## 🟡 Medium

### M1. Simulation skips the real control path
`SwerveModule1507`, `Motor1507` and `Swerve` all branch on `RobotBase.isSimulation()` and return idealized values. Wheels snap to angle instantly, and heading is integrated from the *commanded* omega. The CTRE control requests, gains, current limits and status signals are never exercised, so C1–C3 can't show up in sim.

**Recommended:** drive the CTRE sim state (`TalonFXSimState`, `CANcoderSimState`, `Pigeon2SimState`) from a WPILib `DCMotorSim` per motor, and delete the `isSimulation()` branches in the read path. Then sim tests the same code the robot runs. It also gives us simulated current draw, so we can test the current budget.

### M2. `drive()` and `driveRobotRelative()` are identical
[Swerve.java:232](../src/main/java/org/team1507/robot/subsystems/Swerve.java#L232) says "field-relative" in its javadoc, but it does not convert anything. Callers do the conversion. Pick one name (`driveRobotRelative`) and delete the other.

### M2b. Auto timer only runs if a routine calls `startTimer()`
[AutoSequence](../src/main/java/org/team1507/robot/auto/AutoSequence.java#L460): `driveToBy`, `moveThroughBy` and `waitUntilTime` compare against `autoTimer`, which stays at 0 unless the routine calls `.startTimer()` first. If a student forgets, the cutoffs silently never fire. Start the timer automatically in `AutoOpMode.start()`.

### M3. Teleop default command isn't cancelled when the OpMode ends
[DriverTeleop.end()](../src/main/java/org/team1507/robot/teleop/DriverTeleop.java#L61) calls `removeDefaultCommand()`, which doesn't cancel the drive command that's already running. It gets cancelled today only because the robot disables between modes. Add `robot.swerve.getCurrentCommand().cancel()` (null-checked) to be safe.

### M4. One CAN bus for everything
`Constants.CAN_BUS = CAN_S0` puts every device on one bus. SystemCore has five CAN ports. The hardware can do CAN FD, but [WPILib says the ports currently run CAN 2.0 only](https://github.com/wpilibsuite/SystemCoreTesting/discussions/46), so bandwidth is still tight. Plan on **drivetrain on one port, mechanisms on another**, and log utilization per port (`RobotController.getCANStatus(CANPort)`). A CANivore is still the only path to CAN FD and Phoenix Pro time-synced high-frequency odometry.

### M5. Vision trust equals odometry trust
`VISION_STD_DEV` equals `ODOMETRY_STD_DEV` (0.02 m). Vision is normally trusted *less* than wheel odometry over short times. Revisit this when QuestNav is restored.

### M6. AdvantageKit is installed but unused
It adds a dependency and a naming clash (`LoggedRobot`) without doing anything. Decide in Phase 1: adopt it or remove it.

---

## ⚪ Low

- `Motor1507.getRotorPosition()` integrates simulation state inside a getter, so calling it twice in one loop behaves differently from calling it once. This goes away with M1.
- `Motor1507` stores the motor as `Object` and branches on `instanceof` everywhere. A small interface (or two classes) would be clearer.
- The three identical `applyGains` overloads in `CtreMotorConfigurator` are unavoidable given the Phoenix API, but a test should check they stay in sync.
- Teleop rotation max is `Math.PI` rad/s, while `MAX_ANGULAR_RATE` is 13.1 rad/s. That's fine as a driver preference, but put it in `Constants`.
- The subsystem template declares `package org.wpilib.commands.subsystem1507`. Create one subsystem from the template in WPILib 2027 VS Code to confirm the package line gets rewritten.
- `README.md` describes `lib/core` as "kinematics, vision, mechanics". Update it when the swerve library lands.
- Add a GitHub Actions workflow that runs `./gradlew build` on every push, so a broken `NodeBoundsTest` or compile error is caught before a meeting.

---

## What we do vs. what WPILib 2027 recommends

These were checked against the WPILib 2027 alpha 7 sources installed at `C:\Users\Public\wpilib\2027_alpha7`, not just blog posts.

| Area | 1507Base today | WPILib 2027 / vendor recommendation | Verdict |
|---|---|---|---|
| Robot structure | `OpModeRobot` + `@Autonomous` / `@Teleop` | Same | ✅ Matches |
| Dashboard inputs | `Tunables.publish` | `Tunables` (SmartDashboard removed) | ✅ Matches |
| Telemetry | Custom `Telemetry` → raw NT entries | `org.wpilib.telemetry.Telemetry.log(name, value)`. Publishes under `/Telemetry`, handles structs, units and `TelemetryLoggable` objects. | ⚠️ Replace ours, or rebuild ours on top of it |
| Automatic logging | None | **Epilogue** (`@Logged` on classes and fields), built into WPILib | ⚠️ Adopt |
| Log to disk | `DataLogManager.start()` + `DriverStation.startDataLog` | Same | ✅ Matches |
| Power monitoring | None | `PowerDistribution` (implements `TelemetryLoggable`), `RobotController.getBatteryVoltage()`, `isBrownedOut()`, `setBrownoutVoltages()` | ❌ Missing |
| CAN health | None | `RobotController.getCANStatus(CANPort)` | ❌ Missing |
| CTRE high-rate data | Status signals at 10–100 Hz, logged at 2–50 Hz | CTRE **SignalLogger** (`.hoot`) records every status signal at its own update rate, independent of the robot loop. Convert to `.wpilog` for AdvantageScope. Check current Phoenix licensing terms. | ⚠️ Adopt for matches |
| CTRE configuration | Apply once, print on failure | Retry, check `StatusCode`, keep CANcoder offsets in the device config | ❌ See H3, C3 |
| CTRE bus optimization | Used, but the Pigeon is missed | Set frequencies for **every** signal you read, then optimize | ❌ See C2 |
| Swerve | Custom `SwerveModule1507` + WPILib kinematics / pose estimator | Either CTRE `SwerveDrivetrain` (Tuner X generated, runs its own odometry thread) or a custom stack. Custom is supported, but you own the tuning and data. | ✅ Your choice. Plan below. |
| Commands | Commands v2 vendordep | v2 still ships. **Commands v3** is in the install and conflicts with v2 in the same project. | ⏸ Decide before kickoff |

**On the swerve decision:** CTRE's generated swerve is the lowest-effort path and has a proven high-frequency odometry thread. Its downside is exactly the one you named: it hides the control loop and the data. Owning the swerve is a reasonable choice for a base project *as long as* it gets the things CTRE gets right: device-side offsets, explicit signal rates, sane current limits, and sim that runs the real control path. The plan below builds those in from the start.

---

## Next-steps plan

Phases are ordered by what has to be true before the next one is useful. Each phase is sized in **Wednesday sessions** (2 hours) for a small student group with a mentor. Treat the sizes as rough.

### Phase 0: Critical fixes (1–2 sessions, needs the robot for the last step)
Fixes C1–C3 and H1 so the robot is safe to drive on 2027 code.

1. Configure the CANcoder `MagnetOffset` in code and delete the software offset (C3).
2. Set the Pigeon yaw and yaw-rate frequencies before `optimizeBusUtilization` (C2).
3. Set drive/steer current limits, including the supply lower limit (H1).
4. Add retry and DS error reporting to `CtreMotorConfigurator` (H3).
5. **On the robot:** check wheel alignment, then run SysId on the drive motors in motor-rps units and replace the drive gains (C1). Log before and after, so we have a record of current draw with the old and new gains.

### Phase 1: Brownout-grade data logging (2–3 sessions)
Goal: after any match, answer *"what was the battery voltage, who was pulling current, and what was the robot doing?"*

1. **Pick the stack.** Recommended: WPILib `Telemetry` + Epilogue for robot state, `DataLogManager` for the `.wpilog`, CTRE `SignalLogger` (`.hoot`) for full-rate motor data during matches. Remove AdvantageKit unless you want replay (M6).
2. **Power logging, every loop:** battery voltage, brownout flag, PDH total and per-channel current, CAN utilization per port, loop time.
3. **Motor logging:** supply and stator current at ≥ 50 Hz, plus applied voltage, temperature, and fault flags.
4. Retire or rename our `Telemetry` class so it doesn't collide with WPILib's. Use typed, cached publishers and no per-call `getEntry()`.
5. **Match workflow:** name log files by event and match, save them to USB storage, pull them after every match, and build an AdvantageScope layout with a "brownout view" (voltage + total current + per-motor current on one timeline).
6. A **post-match checklist** students run in the pits: minimum battery voltage, any brownout events, top 3 current consumers, CAN errors.

### Phase 2: 1507 swerve library (4–6 sessions)
Turn `SwerveModule1507` into a proper library we own.

1. **Hardware layer:** a `ModuleIO` interface with a TalonFX/CANcoder implementation and a sim implementation. This is the pattern that makes M1 possible.
2. **Explicit signal plan:** one table listing every signal, its rate and why. Set the rate on each signal, then optimize.
3. **Current budget ("brownout manager"):** watch battery voltage and lower drive stator limits (or scale commanded acceleration) as voltage falls. Log every time it steps in.
4. **Characterization commands** built into the library: SysId for drive and steer, and wheel-radius calibration.
5. **Odometry:** start at 100 Hz on SystemCore CAN 2.0. Decide whether a CANivore and a high-frequency odometry thread are worth it once we see real CAN utilization numbers from Phase 1.
6. Module and chassis telemetry via `@Logged`: desired vs. measured states, per-module current, slip estimate.
7. Fix H4 (driver perspective separate from field heading) and M2 (one drive method).

### Phase 3: Simulation that runs the real code (2–3 sessions)
Implement the sim `ModuleIO` using CTRE sim state + `DCMotorSim`, including a simulated battery (WPILib `BatterySim`). Then sim shows voltage sag and current draw, so current-budget changes can be tested on a laptop at home. Add unit tests for module math and the current budget.

### Phase 4: Framework decisions before kickoff (1–2 sessions)
- Commands v2 or v3: prototype one subsystem in v3 and decide.
- Restore QuestNav when a 2027 build ships (`TODO(QuestNav 2027)`), and revisit the vision std-devs (M5).
- CAN port layout: drivetrain vs. mechanisms (M4).
- GitHub Actions build check; test the subsystem template; README refresh.
- Update to the newest 2027 WPILib / Phoenix / vendor releases and re-run this audit's critical checks.

### Kickoff readiness checklist
- [ ] Robot drives on 2027 code with verified wheel alignment and SysId gains
- [ ] A practice match produces a log with battery, brownout, PDH and per-motor current at ≥ 50 Hz
- [ ] Post-match review takes under 5 minutes with the AdvantageScope layout
- [ ] Sim runs the same swerve code as the robot
- [ ] Every vendordep has a current 2027 release
- [ ] `main` is replaced by `SystemCore`

---

## Appendix A: Comparison with Team 340 (2026)

Source: 340's 2026 robot code (`Rebuilt2026-340`, `robot/subsystems/Swerve.java` and `lib/swerve/`). It's 2026 code, but swerve units and gains don't depend on the WPILib year. Their library applies every gain straight to the TalonFX with **no `SensorToMechanismRatio`**, so:

- **Drive gains** are in volts per **motor** rps (their module sends `m/s × gearRatio / (π × wheelDiameter)`). That's the same units as ours.
- **Turn gains** are in volts per **module** rotation, because the turn motor closes its loop on the CANcoder (`FusedCANcoder` with Pro, `RemoteCANcoder` without). That's also the same units as ours.

So the numbers compare directly.

| Setting | 1507 (ours) | 340 | Notes |
|---|---|---|---|
| Drive gear ratio | 6.122 | 6.027 (675/112) | Different modules, close ratio |
| Steer gear ratio | 21.43 (150/7) | 26.09 (287/11) | Different modules |
| Wheel diameter | 3.904 in | 3.87 in | |
| **Drive kV** (V per motor rps) | **2.75** | **0.125** | 🔴 Ours is 22× too high (C1) |
| Drive kS | 0.12 | 0.0 | Ours is reasonable |
| Drive kA | 0.32 | not used | Our kA is never used: `VelocityVoltage` gets no acceleration |
| **Drive kP** (V per motor rps of error) | **2.0** | **0.25** | 🟠 Ours is 8× higher. Retune after fixing kV. |
| Steer kP (V per module rotation) | 70 | 100 | Same units; ours is a bit softer |
| Steer kD | 0.2 | 0.2 | Same |
| Steer kS / kV | 0.08 / 2.2 | 0 / 0 | 340 uses pure PID on steering |
| Drive stator limit | 120 A (default) | **80 A** | |
| **Drive supply limit** | **70 A** | **28 A** | 🟠 340 limits the four drive motors to 112 A total; ours allows 280 A |
| Steer stator limit | 60 A | 60 A | Same |
| Steer supply limit | 70 A (default) | 40 A | |
| CANcoder offset | Subtracted in software only | Written to `MagnetSensor.MagnetOffset` **in code** | 🔴 340 does what C3 recommends |
| Acceleration limiting | None | Slip limit 15 m/s², torque limit 10 m/s² that shrinks as speed rises | Smooths current spikes. Worth copying (Phase 2). |
| Gyro | Pigeon2, yaw left at 4 Hz (C2) | Redux Canandgyro | |
| Signal reads | Plain values | `getLatencyCompensatedValue(position, velocity)` | Recommended in C2 |
| Odometry | In `periodic()` at 50 Hz | Separate odometry thread at 50 Hz, with a pose history for vision | |
| Odometry std-devs | 0.02, 0.02, 0.05 | 0.1, 0.1, 0.05 | 340 trusts wheels less (see M5) |
| Motor configuration | Apply once, print on failure | Retried through `PhoenixUtil.run`, clear sticky faults first | Matches H3 |
| Phoenix Pro / FOC | Not used | Pro + FOC on all motors | FOC gives more torque per amp. Check whether we have Pro licenses. |
| Logging | Custom NT telemetry | Epilogue `@Logged` on the subsystem | Matches the Phase 1 recommendation |

**Takeaways for Phase 0:**
- Start drive gains near **kS 0.1, kV 0.12, kP 0.1–0.25**, then confirm with SysId.
- Start current limits near 340's: drive **80 A stator / ~30–40 A supply**, steer **60 A stator / ~30–40 A supply**. Drive testing will show whether we need more.
- Steering gains are already in the right range. Don't change them until C3 is fixed and the wheels are verified.
