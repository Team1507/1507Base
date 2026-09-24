# 1507Base

**1507Base** is the production robot codebase for [Team 1507 – Warlocks](https://warlocks1507.com), built on WPILib Java with CTRE Phoenix 6.

> **`SystemCore` branch:** WPILib 2027 (alpha 7) for the NI SystemCore controller. This becomes `main` at kickoff. See [docs/2027-migration.md](docs/2027-migration.md) for what changed from 2026.

## Structure

```
src/main/java/org/team1507/
  robot/
    Constants.java        Tunable parameters (speeds, PID gains, port mappings)
    Robot.java            Subsystems, controllers, driver button bindings
    RobotBehaviors.java   Shared driver/operator input bindings
    auto/routines/        Autonomous OpModes (@Autonomous, one class per routine)
    teleop/               Teleop OpModes (@Teleop)
    subsystems/
      Swerve.java         Swerve drivetrain subsystem
  lib/
    core/                 Shared utilities (kinematics, vision, mechanics)
parked/                   Code waiting on a 2027 vendor library (not compiled)
```

## Setup

1. Install [WPILib 2027 alpha 7](https://github.com/wpilibsuite/SystemcoreTesting) (installs to `C:\Users\Public\wpilib\2027_alpha7`)
2. Open this folder in **WPILib 2027 VS Code** (not the 2026 one)
3. Build: `./gradlew build`
4. Deploy: `./gradlew deploy` (SystemCore must be connected)
5. Simulate: **WPILib: Simulate Robot Code** in VS Code, or `./gradlew run`
6. Pick the auto and teleop modes from the Driver Station drop-downs

When building from a terminal, point Gradle at the 2027 JDK first:
`JAVA_HOME=C:\Users\Public\wpilib\2027_alpha7\jdk`

## License

Uses WPILib components — see [WPILib-License.md](WPILib-License.md).
