//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import org.wpilib.command3.Trigger;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.tunable.TunableBoolean;
import org.wpilib.tunable.Tunables;

import org.team1507.lib.core.util.Alliance;
import org.team1507.robot.auto.nodes.FieldFlip;
import org.team1507.robot.auto.nodes.Nodes;
import org.team1507.robot.subsystems.Swerve;

// ─────────────────────────────────────────────────────────────────────────────
// PoseSeeds
//
// Dashboard buttons that tell the robot where it was placed before a match:
// Seed Left, Seed Center, Seed Right. Each sets the robot's pose to that start
// node (flipped for Red). Works while disabled.
//
// Why: an auto decides which side of the field it is on (and whether to mirror
// the routine) from the robot's pose when it is enabled. With QuestNav tracking,
// the pose is already known. Without it, press the button for where the robot
// actually is. If nothing sets the pose, autos run as written (right side).
//
// In Elastic, add the three /Tunables/Auto/Seed* booleans as toggle buttons.
// Each resets itself to false after seeding.
// ─────────────────────────────────────────────────────────────────────────────
public final class PoseSeeds {

    private PoseSeeds() {}

    /** Creates the three buttons. Called once from Robot.java. */
    public static void bind(Swerve swerve) {
        bind(swerve, "Left",   Nodes.Robot.Start.LEFT);
        bind(swerve, "Center", Nodes.Robot.Start.CENTER);
        bind(swerve, "Right",  Nodes.Robot.Start.RIGHT);
    }

    private static void bind(Swerve swerve, String name, Pose2d bluePose) {
        TunableBoolean button = Tunables.addBoolean("Auto/Seed" + name, false);
        new Trigger(button::get).onTrue(
            swerve.run(coroutine -> {
                swerve.resetPose(Alliance.isRed() ? FieldFlip.pose(bluePose) : bluePose);
                button.set(false);
            }).named("Swerve.seed" + name));
    }
}
