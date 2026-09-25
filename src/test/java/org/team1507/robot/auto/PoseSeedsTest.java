//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.networktables.BooleanEntry;
import org.wpilib.networktables.NetworkTableInstance;
import org.wpilib.tunable.TunableRegistry;

import org.team1507.robot.TestRobot;
import org.team1507.robot.auto.nodes.Nodes;
import org.team1507.robot.subsystems.Swerve;

// ─────────────────────────────────────────────────────────────────────────────
// PoseSeedsTest
//
// Presses the Seed Left dashboard button the way Elastic does (writing true to
// /Tunables/Auto/SeedLeft) and checks the robot's pose moves to the left start
// node, and that the button resets itself.
// ─────────────────────────────────────────────────────────────────────────────
class PoseSeedsTest {

    @Test
    void seedLeftSetsThePoseToTheLeftStart() {
        Swerve swerve = TestRobot.get().swerve;
        TestRobot.setAlliance(false);
        swerve.resetPose(new Pose2d());

        BooleanEntry button = NetworkTableInstance.getDefault()
            .getBooleanTopic("/Tunables/Auto/SeedLeft").getEntry(false);
        button.set(true);                       // the dashboard press

        for (int i = 0; i < 5; i++) {
            TunableRegistry.update();           // the robot loop does this
            Scheduler.getDefault().run();
        }

        Pose2d expected = Nodes.Robot.Start.LEFT;
        assertEquals(expected.getX(), swerve.getPose().getX(), 0.01);
        assertEquals(expected.getY(), swerve.getPose().getY(), 0.01);
        TunableRegistry.update();
        assertFalse(button.get(), "the button should reset itself after seeding");
    }
}
