# 1507Base

**1507Base** is the production robot codebase for [Team 1507 – Warlocks](https://warlocks1507.com), built on WPILib Java with CTRE Phoenix 6.

> **`SystemCore` branch:** WPILib 2027 (alpha 7) for the NI SystemCore controller. This becomes `main` at kickoff. See [docs/2027-migration.md](docs/2027-migration.md) for what changed from 2026.

## Structure

```
src/main/java/org/team1507/
  robot/
    Constants.java        Robot map (CAN IDs, ports) and tuning knobs
    Robot.java            Subsystems, controllers, default commands
    RobotBehaviors.java   Multi-subsystem behaviors (used by teleop and auto)
    auto/routines/        Autonomous OpModes (@Autonomous, one class per routine)
    teleop/               Teleop OpModes (@Teleop) and their button bindings
    subsystems/
      Swerve.java         Swerve drivetrain: odometry, driving, commands
      SwerveConfig.java   Drivetrain hardware: Tuner X paste zone, motor configs, checks
  lib/core/
    framework/            Base classes: LoggedRobot, Subsystem1507, AutoOpMode
    impl/ctre/            Motor1507 and CTRE configuration
    logging/              Telemetry
    swerve/               SwerveModule1507
    util/                 MotorConfig, Alliance
    vision/               VisionConsumer
parked/                   Code waiting on a 2027 vendor library (not compiled)
docs/                     2027 migration story and code audit
```

Uses **Commands v3** (WPILib 2027). Student docs are on the [wiki](https://github.com/Team1507/1507Base/wiki); start with the Season Setup Checklist.

## Setup

1. Install [WPILib 2027 alpha 7](https://github.com/wpilibsuite/SystemcoreTesting) (installs to `C:\Users\Public\wpilib\2027_alpha7`)
2. Open this folder in **WPILib 2027 VS Code** (not the 2026 one)
3. Build: `./gradlew build`
4. Deploy: `./gradlew deploy` (SystemCore must be connected)
5. Simulate: **WPILib: Simulate Robot Code** in VS Code (check **Sim GUI** only), or `./gradlew run`
6. Pick the auto and teleop modes from the Driver Station drop-downs

When building from a terminal, point Gradle at the 2027 JDK first:
`JAVA_HOME=C:\Users\Public\wpilib\2027_alpha7\jdk`

## License

Uses WPILib components — see [WPILib-License.md](WPILib-License.md).
