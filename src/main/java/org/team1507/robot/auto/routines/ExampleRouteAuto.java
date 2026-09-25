//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto.routines;

import org.wpilib.command3.Command;
import org.wpilib.opmode.Autonomous;

import org.team1507.lib.core.framework.AutoOpMode;
import org.team1507.robot.Constants.kAuto.Accuracy;
import org.team1507.robot.auto.AutoSequence;
import org.team1507.robot.auto.nodes.Nodes;

// ─────────────────────────────────────────────────────────────────────────────
// ExampleRouteAuto
//
// A route auto: drive out to midfield and back, then face the hub. Shows every
// kind of path step. Each season, copy it and replace the nodes and actions.
//
// Written for the RIGHT side. Started on the left side (QuestNav, or the Seed
// Left dashboard button), the same routine runs mirrored. Red flips on top.
//
// The commented-out lines show where mechanism actions go: after a checkpoint
// they happen WHILE the robot keeps driving; after an endpoint the robot is
// stopped. Add wrappers for them in AutoSequence's ROBOT ACTIONS section.
// ─────────────────────────────────────────────────────────────────────────────
@Autonomous(name = "Example Route")
public final class ExampleRouteAuto extends AutoOpMode {

    @Override
    protected Command build() {
        return new AutoSequence()                                   // classic driver
            .maxSpeed(0.8)                                          // whole auto at 80% speed
            .resetPose(Nodes.Robot.Start.RIGHT)

            // Out to midfield. The robot drives through the checkpoint without
            // stopping; the step after it happens on the move.
            .checkpoint(Nodes.Robot.Waypoint.MIDFIELD_RIGHT, Accuracy.TIGHT)
            // .intakeDeploy()                                      // happens while driving on

            // Back to score: a waypoint shapes the route (nothing waits for it),
            // then stop at the scoring pose facing the hub.
            .waypoint(Nodes.Robot.Start.RIGHT)
            // .intakeRetract()                                     // (would run here, still driving)
            .slow().endpoint(Nodes.Robot.Score.RIGHT).facing(Nodes.FieldElements.Hub.CENTER)
            // .shootUntil(14.5)                                    // happens while stopped

            .build();
    }
}
