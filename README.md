# 1507Base

**1507Base** is the production robot codebase for [Team 1507 – Warlocks](https://warlocks1507.com), built on WPILib Java with CTRE Phoenix 6.

## Structure

```
src/main/java/org/team1507/
  robot/
    Constants.java        Tunable parameters (speeds, PID gains, port mappings)
    Robot.java            WPILib Robot entry point
    RobotBehaviors.java   Shared driver/operator input bindings
    auto/                 Autonomous routines
    subsystems/
      Swerve.java         Swerve drivetrain subsystem
  lib/
    core/                 Shared utilities (kinematics, vision, mechanics)
```

## Setup

1. Install [WPILib 2026](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
2. Open this folder in VS Code with the WPILib extension
3. Build: `./gradlew build`
4. Deploy: `./gradlew deploy` (robot must be connected)
5. Simulate: `./gradlew simulateJava`

## License

Uses WPILib components — see [WPILib-License.md](WPILib-License.md).
