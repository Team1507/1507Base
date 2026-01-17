// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

//CTRE Imports
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.hardware.TalonFX;

// WPI libraries
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.utilities.Telemetry;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

import static edu.wpi.first.units.Units.*;

import static frc.robot.Constants.Speed.*;

import frc.robot.generated.TunerConstants;

//Subsytem Imports
import frc.robot.subsystems.CommandSwerveDrivetrain;

//Robot Extra
import frc.robot.utilities.Telemetry;

public class RobotContainer {
  public RobotContainer() {
    configureBindings();

  }

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(getMaxSpeed());
    private final CommandXboxController joystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

 
  private void configureBindings() 
  {
    drivetrain.setDefaultCommand(
    drivetrain.applyRequest(() -> {

      double xInput = applyDeadband(-joystick.getLeftY(), 0.15);
      double yInput = applyDeadband(-joystick.getLeftX(), 0.15);
      double rotInput = applyDeadband(-joystick.getRightX(), 0.15);

      return drive
      .withDeadband(getMaxSpeed() * 0.1)
      .withRotationalDeadband(getMaxAngularSpeed() * 0.1)
      .withVelocityX(xInput * getTranslationScale() * getMaxSpeed())
      .withVelocityY(yInput * getTranslationScale() * getMaxSpeed())
      .withRotationalRate(rotInput * getRotationScale() * getMaxAngularSpeed());
    })
  );

    // SysId routines
    joystick.back().and(joystick.y())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x())
        .whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x())
        .whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private double applyDeadband(double value, double deadband) {
        return (Math.abs(value) < deadband) ? 0.0 : value;
    }
}


