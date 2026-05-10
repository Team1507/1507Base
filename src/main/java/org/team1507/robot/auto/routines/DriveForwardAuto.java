//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

import org.team1507.robot.auto.AutoSequence;

// ─────────────────────────────────────────────────────────────────────────────
// DriveForwardAuto
//
// The simplest possible auto routine — resets the pose and drives forward.
// Use this as a reference for how all routine files should look.
//
// To create a new routine:
//   1. Copy this file into the routines/ folder.
//   2. Rename the class and the build() method's steps.
//   3. Register it in Robot.java: autoChooser.addOption("My Auto", MyAuto.build());
// ─────────────────────────────────────────────────────────────────────────────
public final class DriveForwardAuto {

    // Prevent instantiation — call build() directly.
    private DriveForwardAuto() {}

    /**
     * Builds the DriveForward autonomous routine.
     *
     * Steps:
     *   1. Reset pose to field origin (0, 0, 0°).
     *   2. Drive 5 m forward at full speed — tests APF deceleration.
     *   3. Stop.
     */
    public static Command build() {
        return new AutoSequence()
            .resetPose(new Pose2d())
            .driveForwardMeters(5.0, true)
            .stop()
            .build();
    }
}