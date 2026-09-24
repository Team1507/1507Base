# Migrating 1507Base to WPILib 2027 (SystemCore)

This is the story of how 1507Base moved from the 2026 roboRIO to the 2027 NI SystemCore controller. Every section matches one commit on the `SystemCore` branch, so you can follow along with `git log` and see exactly what changed:

```
git log --oneline main..SystemCore
```

2027 is the biggest control-system change since 2015. Almost every file changed. The good news is that most of the changes are mechanical, and the compiler tells you about them.

---

## 1. Project files (`Update project files to WPILib 2027 alpha 7`)

A WPILib project is more than Java code. The **build files** tell Gradle which WPILib version to download, which Java version to use, and where to deploy.

| File | 2026 | 2027 |
|---|---|---|
| `build.gradle` plugin | `edu.wpi.first.GradleRIO` 2026.2.1 | `org.wpilib.GradleRIO` 2027.0.0-alpha-7 |
| Java version | 17 | **25** |
| Deploy target | `roborio` | `systemcore` |
| Deploy folder on robot | `/home/lvuser/deploy` | `/home/systemcore/deploy` |
| Gradle | 8.11 | 9.4.1 |
| `.wpilib/wpilib_preferences.json` | `"projectYear": "2026"` | `"projectYear": "2027_alpha7"` |

**How we did it:** we copied these files from the official 2027 project template that ships inside the WPILib VS Code extension, rather than hand-editing the old ones. One 1507 change was kept: `includeDesktopSupport = true`, so simulation works on laptops.

## 2. Vendor libraries (`Swap vendor libraries for 2027 alpha 7 versions`)

Vendor libraries (vendordeps) are code from other companies: CTRE for motors, AdvantageKit for logging. **Each one must be rebuilt for 2027**, because WPILib renamed all of its packages (see step 3). A 2026 vendordep will not work.

We checked WPILib's official compatibility table ([SystemcoreTesting](https://github.com/wpilibsuite/SystemcoreTesting)) for each library:

| Library | Result |
|---|---|
| CTRE Phoenix 6 | ✅ `26.70.0-alpha-2` |
| AdvantageKit | ✅ `27.0.0-alpha-5`. Later removed: logging uses WPILib's built-in Telemetry instead. |
| PathPlanner | ❌ Removed. We're building our own path policy. |
| QuestNav | ⏸️ No 2027 build yet, so it's parked (see step 6). |

**Lesson:** before a season, check that *every* vendor library you depend on supports the new year.

## 3. The big rename (`Rename WPILib packages edu.wpi.first -> org.wpilib`)

WPILib moved every class to a new package name:

```java
// 2026
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

// 2027
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.Command;
```

Some classes also got new names while moving:

| 2026 | 2027 |
|---|---|
| `ChassisSpeeds` | `ChassisVelocities` |
| `speeds.omegaRadiansPerSecond` | `speeds.omega` |
| `Timer.getFPGATimestamp()` | `Timer.getTimestamp()` |
| `DriverStation.getAlliance()` | `MatchState.getAlliance()` |
| `Alliance.Blue` / `Alliance.Red` | `Alliance.BLUE` / `Alliance.RED` |

**How we did it:** WPILib's VS Code importer has a list of about 400 find-and-replace rules. We ran those exact rules, in the same order, over our code. That handled about 200 lines across 26 files automatically.

## 4. Removing PathPlanner (`Remove PathPlanner`)

We deleted the PathPlanner setup in `AutoBuilder`, the `drivePath()` steps in `AutoSequence`, and the `deploy/pathplanner/` folder.

## 5. Parking QuestNav (`Park QuestNav until a 2027 build ships`)

QuestNav's library is still built against the old `edu.wpi.first` packages, so it can't run on 2027. Instead of deleting our QuestNav code, we moved it to `parked/QuestNavSubsystem.java.txt`. The `.txt` ending and the location outside `src/` mean it is not compiled. The top of that file explains how to bring it back. Search the code for `TODO(QuestNav 2027)` to find every spot that needs it.

## 6. Fixing what the compiler found (`Fix 2027 API changes found by the compiler`)

After the rename, the build still had **44 errors**. This is normal. Read each error, find what the new API is, and fix it. Here's what we hit:

### CTRE devices need a CAN bus
SystemCore has **five CAN ports** (`CAN_S0` to `CAN_S4`) instead of the roboRIO's one. Every TalonFX, CANcoder and Pigeon2 must say which one it's on:

```java
// Constants.java
public static final CANBus CAN_BUS = new CANBus(CANPort.CAN_S0);

// 2026                               // 2027
new CANcoder(RobotMap.FL_ENCODER)     new CANcoder(RobotMap.FL_ENCODER, Constants.CAN_BUS)
```

`Motor1507` now takes the bus too: `new Motor1507(name, Type.FX, id, Constants.CAN_BUS, config)`.

### Swerve math is now *immutable* ⚠️ read this one carefully
In 2026, some math methods **changed the object you called them on**. In 2027 they **return a new object** and leave the original alone. The dangerous part is that the old code still compiles; it just silently does nothing:

```java
// 2026: optimize() changed `state` in place
state.optimize(currentAngle);

// 2027: that same line compiles, but throws the result away!
state = state.optimize(currentAngle);   // must assign it back
```

The same applies to `desaturateWheelVelocities()`, which now returns a new array. **The compiler cannot catch this.** You have to know it changed. That's why reading the [yearly changelog](https://docs.wpilib.org) matters.

### Other changes
- `SwerveModuleState` → `SwerveModuleVelocity`
- `ChassisSpeeds.fromFieldRelativeSpeeds(x, y, omega, heading)` → `new ChassisVelocities(x, y, omega).toRobotRelative(heading)`
- `Translation2d.getAngle()` now returns an `Optional`, because a zero-length arrow has no direction.
- `Rotation2d.kZero` → `Rotation2d.ZERO`
- `Rotation2d.getDegrees()` etc. now wrap to -180°..180°. We checked every use; ours were already safe.

### SmartDashboard is gone
SmartDashboard, `SendableChooser` and Shuffleboard were removed. Dashboard values and buttons now go through **Tunables**:

```java
// 2026
SmartDashboard.putData("Set Pose Left", command);
// 2027
Tunables.publish("Set Pose Left", command);
```

## 7. OpModes (`Convert 1507Base to the 2027 OpMode framework`)

This is the biggest *design* change. In 2026 we picked autos from a dashboard chooser, and `Robot.java` had `autonomousInit()` and `teleopInit()`. In 2027, **each mode is its own class**, and the Driver Station shows drop-downs to pick them (similar to FTC):

```java
@Autonomous(name = "Drive Forward")
public final class DriveForwardAuto extends AutoOpMode {
    @Override
    protected Command build() {
        return new AutoSequence()
            .resetPose(new Pose2d())
            .driveForwardMeters(5.0, true)
            .stop()
            .build();
    }
}
```

**To add a new auto:** copy `DriveForwardAuto.java`, then rename the class and the `@Autonomous` name. That's it. There's no list to register it in. WPILib finds it automatically.

How the pieces fit:

| File | Job |
|---|---|
| `Robot.java` | Owns subsystems, controllers and button bindings that work in every mode |
| `lib/core/framework/AutoOpMode.java` | Base class for autos. Starts the command when enabled and cancels it when disabled. |
| `robot/auto/routines/*.java` | One `@Autonomous` class per auto routine |
| `robot/teleop/DriverTeleop.java` | `@Teleop` mode. Gives the driver joystick control of swerve. |

**Why button bindings stay in `Robot.java`:** with Commands v2, a binding can never be removed. If an OpMode created bindings, then every time you selected it again, it would add *another* copy.

> **Update (Commands v3):** 1507Base has since moved to Commands v3 (see step 8). v3 ties bindings to the OpMode that creates them and removes them automatically, so the driver bindings now live in `DriverTeleop`.

The Xbox controller class was also replaced by a generic `CommandGamepad`. Buttons are named by position: `a()` is now `faceDown()`.

---

## 8. Commands v3 (`Move to Commands v3`)

WPILib 2027 ships a new command framework, **Commands v3**, alongside the old one (v2). A project can use only one. We moved to v3 even though it is still alpha and may change, because it records far more about every command (useful for match debugging) and fits the OpMode framework.

| v2 | v3 |
|---|---|
| `SubsystemBase` | `Mechanism` (our `Subsystem1507` implements it) |
| `run(() -> ...)` / `runOnce(...)` | `runRepeatedly(() -> ...)` / `run(coroutine -> ...)` |
| `initialize` / `execute` / `isFinished` / `end` | One function that reads top to bottom, with `coroutine.yield()` in every loop |
| `.finallyDo(...)` | Code after the loop (finished) + `.whenCanceled(...)` (interrupted) |
| `.withName(...)` optional | `.named(...)` **required** on every command |
| `Commands.sequence/parallel/race/deadline` | `Command.sequence/parallel/race`, deadline = `Command.parallel(d).optional(others)` |
| Bindings live forever | Bindings made in an OpMode are removed when another OpMode is selected |
| Commands canceled on disable | Not automatic in v3; `LoggedRobot` cancels everything on disable |
| `CommandBuilder` (ours) | Removed. v3 covers it. |

The one v3 trap: **a loop inside a command must call `coroutine.yield()`**, or the whole robot program freezes. For "do this every loop until interrupted," use `runRepeatedly(...)`, which yields for you.

`CommandsV3PatternsTest` runs the patterns we rely on through the real v3 scheduler, so a WPILib update that changes their behavior fails the build.

## Still to do

- **QuestNav**: restore it when a 2027 build ships (expected at kickoff).
- **Data logging**: next project. Built on WPILib's new Telemetry, with battery, current and command logging to track down brownouts. (AdvantageKit was compared and removed: we don't need its replay feature, and it would add structure students must remember.)
- **CAN port**: confirm which physical SystemCore port (`CAN_S0`–`CAN_S4`) the swerve is wired to, and update `Constants.CAN_BUS`.

## Checklist: doing this yourself next year

1. Install the new WPILib and read the **yearly changelog** and **removed features** pages.
2. Make a new branch. Never migrate on `main`.
3. Check that every vendor library has a build for the new year.
4. Copy the build files from the new project template.
5. Run the importer's rename rules (or use the VS Code importer).
6. Build, then fix errors one group at a time and commit each group.
7. Search for **silent** changes that the compiler can't catch (like the immutable math above).
8. Run the unit tests, then run in simulation.
9. Write down what you changed, like this file.
