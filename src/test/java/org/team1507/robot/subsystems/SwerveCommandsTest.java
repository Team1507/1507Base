//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;

import org.team1507.robot.TestRobot;

// ─────────────────────────────────────────────────────────────────────────────
// SwerveCommandsTest
//
// Builds the real Swerve subsystem on CTRE's simulated hardware and runs its
// commands through the Commands v3 scheduler, checking they end where they
// should. Protects the driving commands when Swerve.java is refactored.
// (Auto routes, which replaced driveToPoint / moveThroughPose, are tested in
// AutoSequenceTest.)
//
// Simulation here is simple (wheels reach their commanded speed instantly), so
// these check the command LOGIC, not real-robot tuning.
// ─────────────────────────────────────────────────────────────────────────────
class SwerveCommandsTest {

    private static Swerve swerve;
    private static final Scheduler scheduler = Scheduler.getDefault();

    /** Stops a runaway command from hanging the build. */
    private static final int MAX_LOOPS = 1500;   // 30 simulated seconds

    @BeforeAll
    static void createSwerve() {
        swerve = TestRobot.get().swerve;   // the shared simulated robot's drivetrain
    }

    @BeforeEach
    void reset() {
        scheduler.cancelAll();
        swerve.resetPose(new Pose2d());
    }

    /** Runs the scheduler until the command finishes; returns the loops it took. */
    private static int runUntilDone(Command command) {
        scheduler.schedule(command);
        for (int loop = 1; loop <= MAX_LOOPS; loop++) {
            scheduler.run();
            if (!scheduler.isScheduledOrRunning(command)) {
                return loop;
            }
        }
        throw new AssertionError(command.name() + " did not finish in " + MAX_LOOPS + " loops");
    }

    @Test
    void driveForwardMetersEndsTheRightDistanceAhead() {
        runUntilDone(swerve.driveForwardMeters(1.0, 2.0, true));
        Pose2d pose = swerve.getPose();
        assertEquals(1.0, pose.getX(), 0.06, "should stop within ARRIVE_THRESHOLD of 1 m ahead");
        assertEquals(0.0, pose.getY(), 0.06);
    }

    @Test
    void changeHeadingEndsFacingTheTarget() {
        runUntilDone(swerve.changeHeading(90));
        double headingDeg = swerve.getPose().getRotation().getDegrees();
        assertEquals(90.0, headingDeg, 3.5, "within HEADING_TOLERANCE_DEG");
    }

    @Test
    void pointToTargetFacesTheFieldPosition() {
        // A target straight to the robot's left: it should turn to +90°.
        runUntilDone(swerve.pointToTarget(new Pose2d(0.0, 2.0, Rotation2d.ZERO)));
        assertEquals(90.0, swerve.getPose().getRotation().getDegrees(), 3.5);
    }

    @Test
    void stoppedDriveCommandDoesNotMoveTheRobot() {
        Command drive = swerve.driveForwardMeters(3.0, 2.0, true);
        scheduler.schedule(drive);
        for (int i = 0; i < 10; i++) scheduler.run();
        scheduler.cancel(drive);
        // In simulation the pose lags the wheels by one loop, so let that last
        // step land before measuring.
        for (int i = 0; i < 2; i++) scheduler.run();
        double xAfterStop = swerve.getPose().getX();
        for (int i = 0; i < 20; i++) scheduler.run();
        assertFalse(scheduler.isScheduledOrRunning(drive));
        assertEquals(xAfterStop, swerve.getPose().getX(), 1e-6, "robot should stay stopped once canceled");
    }
}
