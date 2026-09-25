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
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.AfterAll;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import org.wpilib.command3.Command;
import org.wpilib.command3.Scheduler;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;

import org.team1507.lib.core.util.Alliance;
import org.team1507.robot.Constants.kAuto;
import org.team1507.robot.Constants.kAuto.Accuracy;
import org.team1507.robot.TestRobot;
import org.team1507.robot.auto.nodes.Nodes;
import org.team1507.robot.subsystems.Swerve;

// ─────────────────────────────────────────────────────────────────────────────
// AutoSequenceTest
//
// Runs auto routes on the simulated robot and checks the behavior the 2027 auto
// plan promises (docs/2027-auto-plan.md):
//   - after a checkpoint, the next steps run WHILE the robot keeps driving
//   - after an endpoint, the robot is stopped
//   - .facing() / .heading() make an endpoint wait for the heading
//   - mirroring to the left side, and Red flipping
//   - .by() time cutoffs
//   - .holdUntil() keeps the robot on a checkpoint until a condition is true
//   - keep-running (background) actions stop when the auto ends
//   - canceling the auto stops the route
//   - build() catches mistakes before the robot moves
//   - the route check flags a classic leg that crosses the hub
//
// The simulation is simple (wheels reach their speed at once), so this checks
// the auto LOGIC, not real-robot tuning.
// ─────────────────────────────────────────────────────────────────────────────
class AutoSequenceTest {

    private static final Scheduler scheduler = Scheduler.getDefault();
    private static final int MAX_LOOPS = 1500;   // 30 simulated seconds
    private static Swerve swerve;

    @BeforeAll
    static void setUp() {
        swerve = TestRobot.get().swerve;
        TestRobot.pauseTime();
    }

    @AfterAll
    static void tearDown() {
        TestRobot.resumeTime();
    }

    @BeforeEach
    void reset() {
        scheduler.cancelAll();
        TestRobot.setAlliance(false);
        swerve.resetPose(new Pose2d());
        TestRobot.step(2);
    }

    /** Runs an auto to completion; returns the loops it took. */
    private static int run(Command auto) {
        scheduler.schedule(auto);
        for (int loop = 1; loop <= MAX_LOOPS; loop++) {
            TestRobot.step();
            if (!scheduler.isScheduledOrRunning(auto)) {
                return loop;
            }
        }
        throw new AssertionError(auto.name() + " did not finish in " + MAX_LOOPS + " loops");
    }

    /** A step that takes 10 loops and records how far the robot moved meanwhile. */
    private static Command probe(double[] movedMeters) {
        return Command.noRequirements(coroutine -> {
            Translation2d start = swerve.getPose().getTranslation();
            for (int i = 0; i < 10; i++) {
                coroutine.yield();
            }
            movedMeters[0] = swerve.getPose().getTranslation().getDistance(start);
        }).named("probe");
    }

    private static Pose2d node(double x, double y, double degrees) {
        return new Pose2d(x, y, Rotation2d.fromDegrees(degrees));
    }

    private static void assertAt(double x, double y, double tolerance) {
        Pose2d pose = swerve.getPose();
        assertEquals(x, pose.getX(), tolerance, "x");
        assertEquals(y, pose.getY(), tolerance, "y");
    }

    // ── Driving ──────────────────────────────────────────────────────────────

    @Test
    void stepsAfterACheckpointRunWhileTheRobotKeepsDriving() {
        double[] moved = {0};
        run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .checkpoint(node(2, 1, 0))
            .addCommand(probe(moved))
            .endpoint(node(4, 1, 0))
            .build());
        assertTrue(moved[0] > 0.3, "robot should keep driving during the step after a checkpoint; moved " + moved[0]);
        assertAt(4, 1, 0.06);
    }

    @Test
    void stepsAfterAnEndpointRunWhileStopped() {
        double[] moved = {1};
        run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .endpoint(node(2, 1, 0))
            .addCommand(probe(moved))
            .build());
        assertEquals(0.0, moved[0], 0.02, "robot should be stopped after an endpoint");
        assertAt(2, 1, 0.06);
    }

    @Test
    void facingMakesTheEndpointFaceTheLocation() {
        run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .endpoint(node(3, 1, 0)).facing(new Translation2d(3, 3))   // straight to the left
            .build());
        assertAt(3, 1, 0.06);
        assertEquals(90.0, swerve.getPose().getRotation().getDegrees(), 3.5);
    }

    @Test
    void headingOverridesTheNodeHeadingAndWaits() {
        run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .endpoint(node(2, 1, 0)).heading(-120)
            .build());
        assertEquals(-120.0, swerve.getPose().getRotation().getDegrees(), 3.5);
    }

    @Test
    void checkpointsWithAccuracyPresetsAreReached() {
        run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .checkpoint(node(2, 1.5, 0), Accuracy.PRECISE)
            .checkpoint(node(3, 1, 0), Accuracy.LOOSE)
            .waypoint(node(4, 1.2, 0))
            .endpoint(node(5, 1, 0))
            .build());
        assertAt(5, 1, 0.06);
    }

    @Test
    void timeCutoffMovesOnFromANodeTheRobotCantReachInTime() {
        int loops = run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .checkpoint(node(8, 1, 0)).by(0.3)     // 7 m away: can't make it in 0.3 s
            .endpoint(node(1.5, 1, 0))
            .build());
        assertAt(1.5, 1, 0.06);
        assertTrue(loops < 200, "should not drive all the way to x = 8 first; took " + loops + " loops");
    }

    @Test
    void holdUntilKeepsTheRobotOnTheCheckpointUntilTheConditionIsTrue() {
        boolean[] released = {false};
        double[] distanceWhenDone = {-1};
        // A background action that finishes after 60 loops, like an intake deploying.
        Command deploy = Command.noRequirements(coroutine -> {
            for (int i = 0; i < 60; i++) coroutine.yield();
            released[0] = true;
            coroutine.park();
        }).named("fake deploy");
        Command probe = Command.noRequirements(coroutine ->
            distanceWhenDone[0] = swerve.getPose().getTranslation().getDistance(new Translation2d(1.6, 1))
        ).named("probe");

        int loops = run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .runInBackground("fakeDeploy", deploy)
            .checkpoint(node(1.6, 1, 0)).holdUntil(() -> released[0])
            .addCommand(probe)
            .endpoint(node(3, 1, 0))
            .build());

        assertTrue(loops > 60, "should wait for the hold; took " + loops + " loops");
        assertTrue(distanceWhenDone[0] >= 0 && distanceWhenDone[0] < 0.4,
            "robot should be on the checkpoint when the hold releases: " + distanceWhenDone[0] + " m");
        assertAt(3, 1, 0.06);
    }

    @Test
    void aHoldThatNeverReleasesTimesOutAndTheAutoContinues() {
        int loops = run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .checkpoint(node(1.6, 1, 0)).holdUntil(() -> false)
            .endpoint(node(3, 1, 0))
            .build());
        assertAt(3, 1, 0.06);
        assertTrue(loops > kAuto.HOLD_TIMEOUT_SECONDS * 50, "should wait the hold timeout first; took " + loops);
    }

    // ── Where the routine runs ───────────────────────────────────────────────

    @Test
    void leftSideMirrorsEveryNode() {
        run(new AutoSequence()
            .side(Side.LEFT)
            .resetPose(node(1, 1, 0))
            .endpoint(node(2, 1, 30)).heading()
            .build());
        assertAt(2, Nodes.Field.WIDTH - 1, 0.06);
        assertEquals(-30.0, swerve.getPose().getRotation().getDegrees(), 3.5);
    }

    @Test
    void redAllianceFlipsEveryNode() {
        TestRobot.setAlliance(true);
        assertTrue(Alliance.isRed(), "test setup: alliance should be Red");
        run(new AutoSequence()
            .noMirror()
            .resetPose(node(1, 1, 0))
            .endpoint(node(2, 1, 0)).heading()
            .build());
        assertAt(Nodes.Field.LENGTH - 2, Nodes.Field.WIDTH - 1, 0.06);
        assertEquals(180.0, Math.abs(swerve.getPose().getRotation().getDegrees()), 3.5);
    }

    // ── Background actions and canceling ─────────────────────────────────────

    @Test
    void keepRunningActionsStopWhenTheAutoEnds() {
        boolean[] running = {false};
        boolean[] stopped = {false};
        Command rollers = Command.noRequirements(coroutine -> {
            running[0] = true;
            coroutine.park();
        }).whenCanceled(() -> stopped[0] = true).named("rollers");

        run(new AutoSequence()
            .runInBackground("rollers", rollers)
            .waitSeconds(0.2)
            .build());
        assertTrue(running[0], "keep-running action should have started");
        assertTrue(stopped[0], "keep-running action should stop when the auto ends");
    }

    @Test
    void cancelingTheAutoStopsTheRobot() {
        Command auto = new AutoSequence()
            .resetPose(node(1, 1, 0))
            .endpoint(node(6, 1, 0))
            .build();
        scheduler.schedule(auto);
        TestRobot.step(20);
        scheduler.cancel(auto);          // what happens when the robot disables
        TestRobot.step(2);
        double x = swerve.getPose().getX();
        TestRobot.step(20);
        assertFalse(scheduler.isScheduledOrRunning(auto));
        assertEquals(x, swerve.getPose().getX(), 1e-6, "robot should stay stopped");
    }

    // ── Mistakes caught by build() ───────────────────────────────────────────

    private static void assertMistake(String expected, AutoSequence routine) {
        IllegalStateException e = assertThrows(IllegalStateException.class, routine::build);
        assertTrue(e.getMessage().contains(expected), "message should mention \"" + expected + "\": " + e.getMessage());
    }

    @Test
    void aDrivetrainStepWhileTheRouteDrivesIsAMistake() {
        assertMistake("uses the drivetrain while the robot is still driving", new AutoSequence()
            .checkpoint(node(2, 1, 0))
            .changeHeading(90)
            .endpoint(node(4, 1, 0)));
    }

    @Test
    void aDrivetrainStepAfterTheEndpointIsFine() {
        run(new AutoSequence()
            .resetPose(node(1, 1, 0))
            .checkpoint(node(2, 1, 0))
            .endpoint(node(3, 1, 0))
            .changeHeading(90)
            .build());
        assertEquals(90.0, swerve.getPose().getRotation().getDegrees(), 3.5);
    }

    @Test
    void aMisplacedSpeedModifierIsAMistake() {
        assertMistake("does nothing", new AutoSequence()
            .slow().waitSeconds(1)
            .endpoint(node(2, 1, 0)));
    }

    @Test
    void aHeadingModifierAfterAnActionIsAMistake() {
        assertMistake("must come right after a checkpoint", new AutoSequence()
            .endpoint(node(2, 1, 0))
            .waitSeconds(1).heading());
    }

    @Test
    void aWaypointAtTheEndIsAMistake() {
        assertMistake("never drive to it", new AutoSequence()
            .endpoint(node(2, 1, 0))
            .waypoint(node(3, 1, 0)));
    }

    @Test
    void aPolicyAutoWithoutThePolicyIsAMistake() {
        assertMistake("policy driver", new AutoSequence(Driver.POLICY)
            .endpoint(node(2, 1, 0)));
    }

    // ── Route checks ─────────────────────────────────────────────────────────

    @Test
    void aClassicLegThroughTheHubIsFlagged() {
        @SuppressWarnings("unused")
        Command built = new AutoSequence()
            .resetPose(node(2, 4.1, 0))
            .endpoint(node(5.5, 4.1, 0))    // straight through the hub
            .build();
        assertFalse(AutoSequence.warningsFromLastBuild().isEmpty(), "leg through the hub should be flagged");
        assertTrue(AutoSequence.warningsFromLastBuild().get(0).contains("FieldElements.Hub"));
    }

    @Test
    void aClassicLegClearOfTheHubIsNotFlagged() {
        @SuppressWarnings("unused")
        Command built = new AutoSequence()
            .resetPose(node(2, 2, 0))
            .endpoint(node(5.5, 2, 0))      // 1.5 m below the hub
            .build();
        assertTrue(AutoSequence.warningsFromLastBuild().isEmpty(), AutoSequence.warningsFromLastBuild().toString());
    }
}
