//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command3.Command;
import org.wpilib.opmode.Autonomous;

import org.team1507.lib.core.framework.AutoOpMode;
import org.team1507.robot.auto.AutoSequence;

// ─────────────────────────────────────────────────────────────────────────────
// DriveForwardAuto
//
// The simplest possible auto routine — resets the pose and drives forward.
// Use this as a reference for how all routine files should look.
//
// To create a new routine:
//   1. Copy this file into the routines/ folder.
//   2. Rename the class, change the @Autonomous name, and edit build()'s steps.
//   That's it — the Driver Station lists every @Autonomous class automatically.
// ─────────────────────────────────────────────────────────────────────────────
@Autonomous(name = "Drive Forward")
public final class DriveForwardAuto extends AutoOpMode {

    /**
     * Builds the DriveForward autonomous routine.
     *
     * Steps:
     *   1. Reset pose to field origin (0, 0, 0°).
     *   2. Drive 5 m forward at full speed — tests APF deceleration.
     *   3. Stop.
     */
    @Override
    protected Command build() {
        return new AutoSequence()
            .resetPose(new Pose2d())
            .driveForwardMeters(5.0, true)
            .stop()
            .build();
    }
}