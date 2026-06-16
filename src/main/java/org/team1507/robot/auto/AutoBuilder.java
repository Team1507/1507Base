//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╔╝╚██╝  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import org.team1507.lib.core.util.Alliance;
import org.team1507.robot.subsystems.Swerve;

// ─────────────────────────────────────────────────────────────────────────────
// AutoBuilder
//
// Static registry that holds all subsystem references and exposes them as
// zero-argument command factories. Initialized ONCE from Robot.java after
// all subsystems are created.
//
// HOW TO ADD A NEW SUBSYSTEM EACH YEAR:
//   1. Import your subsystem at the top of this file.
//   2. Add a public static field for it below the existing fields.
//   3. Add it as a parameter to init() and assign it.
//   4. Add your command factory methods in the appropriate section below.
//   5. Call AutoBuilder.init(...) in Robot.java with the new subsystem.
//
// Students should NEVER instantiate this class. Always call AutoBuilder.method().
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoBuilder {

    // -------------------------------------------------------------------------
    // Subsystem Registry
    // -------------------------------------------------------------------------

    public static Swerve swerve;

    // ADD NEW SUBSYSTEMS HERE each year (e.g. climber, indexer, turret).

    // -------------------------------------------------------------------------
    // Initialization
    //
    // Called ONCE from Robot.java constructor, after all subsystems are built.
    // Add new subsystem parameters here as the robot grows each year.
    // -------------------------------------------------------------------------

    public static void init(Swerve swerve) {
        AutoBuilder.swerve = swerve;

        // Configure PathPlanner once. Uses fully-qualified class name to avoid conflict
        // with this class (both are named AutoBuilder in different packages).
        try {
            com.pathplanner.lib.auto.AutoBuilder.configure(
                swerve::getPose,
                swerve::resetPose,
                swerve::getChassisSpeeds,
                (speeds, feedforwards) -> swerve.driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                    new PIDConstants(5.0, 0.0, 0.0),  // translation PID — tune in sim
                    new PIDConstants(5.0, 0.0, 0.0)   // rotation PID — tune in sim
                ),
                RobotConfig.fromGUISettings(),
                () -> Alliance.isRed(),
                swerve
            );
        } catch (Exception e) {
            throw new RuntimeException("PathPlanner AutoBuilder.configure() failed — check deploy/pathplanner/settings.json", e);
        }
    }

    // Prevent instantiation — this is a static utility class.
    private AutoBuilder() {}
}