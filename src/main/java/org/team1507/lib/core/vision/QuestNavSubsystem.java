//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.vision;

import java.util.function.Consumer;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

import org.team1507.lib.core.framework.Subsystem1507;
import org.team1507.lib.core.util.CommandBuilder;

// ─────────────────────────────────────────────────────────────────────────────
// QuestNavSubsystem
//
// Reads pose data from a Meta Quest running QuestNav and feeds it into the
// swerve drive's pose estimator.
//
// ── Pre-match pose preset workflow ──────────────────────────────────────────
//   1. Robot is placed at a known starting position on the field.
//   2. Driver presses "Set Pose Left/Start/Right" on Elastic.
//   3. setKnownPoseCommand sends the pose to QuestNav and waits for confirmation.
//   4. Only after the Quest confirms does odometry snap to the new pose.
//
//   This replaces the 2026 approach of mashing the button repeatedly — the
//   command now blocks until the Quest acknowledges the reset (or times out).
//
// ── Wiring in Robot.java ────────────────────────────────────────────────────
//   questNav = new QuestNavSubsystem(
//       swerve::addVisionMeasurement,
//       swerve::resetPose,
//       Constants.kQuest.ROBOT_TO_QUEST   // transform from robot center → Quest mount
//   );
//
//   // Dashboard buttons (pre-match, runs when disabled):
//   questNav.setKnownPoseCommand(Nodes.Robot.Start.LEFT)
//       .named("Set Pose Left").publishToDashboard();
//   questNav.setKnownPoseCommand(Nodes.Robot.Start.CENTER)
//       .named("Set Pose Start").publishToDashboard();
//   questNav.setKnownPoseCommand(Nodes.Robot.Start.RIGHT)
//       .named("Set Pose Right").publishToDashboard();
//
// ── Mounting offset ─────────────────────────────────────────────────────────
//   The Quest reports its OWN 3D pose, not the robot's center pose.
//   Supply a Transform3d from robot center to the Quest's physical mount point.
//   Leave as Transform3d.kZero if the Quest is mounted at the center.
//   Last year's offset: Transform3d(-0.202, 0.304, 0.39, Rotation3d(0, 0, 90°))
//
// ── onCommandSuccess / onCommandFailure ─────────────────────────────────────
//   These callbacks are new in QuestNavLib 2026-2.2.0. They fire inside
//   commandPeriodic() when the Quest headset acknowledges or rejects a setPose
//   command. They drive setKnownPoseCommand's completion — no polling needed.
//
//   Both callbacks pass a Commands.ProtobufQuestNavCommandResponse, so the
//   lambda form is `response ->` (Consumer), not `() ->` (Runnable).
//   The response object is ignored — success/failure state is all we need.
// ─────────────────────────────────────────────────────────────────────────────
public final class QuestNavSubsystem extends Subsystem1507 {

    // =========================================================================
    // Tuning
    // =========================================================================

    /**
     * Confidence for vision measurements: [x (m), y (m), heading (rad)].
     * Lower values = trust QuestNav more relative to wheel odometry.
     * Tightened from the initial 0.1/0.1/0.05 based on 2026 field results.
     * Store in Constants.kQuest if you need to tune per-robot.
     */
    private static final double[] STD_DEVS = { 0.02, 0.02, 0.05 };

    /**
     * How long setKnownPoseCommand waits for Quest acknowledgment before giving up.
     * 3 seconds is conservative — typical response time is under 100 ms when tracking.
     */
    private static final double SET_POSE_TIMEOUT_SECONDS = 3.0;

    // =========================================================================
    // Callbacks passed in from Robot.java
    // =========================================================================

    /** Feeds a vision pose into the drivetrain's pose estimator. */
    private final VisionConsumer onMeasurement;

    /** Resets the robot's odometry pose directly. */
    private final Consumer<Pose2d> onReset;

    // =========================================================================
    // QuestNav library
    // =========================================================================

    private final QuestNav questNav = new QuestNav();

    /**
     * Transform from robot center to the Quest's physical mounting point.
     * Applied before sending poses to the Quest (robot center → Quest mount).
     * Applied in reverse when converting Quest poses back to robot center.
     */
    private final Transform3d mountingOffset;

    // =========================================================================
    // Pose reset state
    // =========================================================================

    /**
     * True while a setPose command is in-flight to the Quest.
     * Set to true in setKnownPoseCommand's initialize(); cleared by
     * onCommandSuccess or onCommandFailure (both fire inside commandPeriodic()).
     */
    private boolean poseResetPending = false;

    /**
     * Set to true by onCommandSuccess when the Quest confirms it accepted the pose.
     * Read by setKnownPoseCommand's onEnd to decide whether to snap odometry.
     */
    private boolean poseResetSucceeded = false;

    /**
     * True once QuestNav has produced at least one valid tracking frame.
     * Guards against sending pose commands before the Quest is fully ready —
     * a lesson from 2026 when stale commands were silently rejected.
     */
    private boolean hasSeenFreshFrame = false;

    // =========================================================================
    // Constructors
    // =========================================================================

    /**
     * Creates a QuestNavSubsystem assuming the Quest is mounted at the robot's center.
     *
     * @param onMeasurement called each loop with a fresh pose estimate — pass
     *                      {@code swerve::addVisionMeasurement}
     * @param onReset       called when resetting odometry from a known pose — pass
     *                      {@code swerve::resetPose}
     */
    public QuestNavSubsystem(VisionConsumer onMeasurement, Consumer<Pose2d> onReset) {
        this(onMeasurement, onReset, Transform3d.kZero);
    }

    /**
     * Creates a QuestNavSubsystem with a mounting offset from the robot's center.
     *
     * @param onMeasurement  called each loop with a fresh pose estimate
     * @param onReset        called when resetting odometry from a known pose
     * @param mountingOffset transform from robot center to the Quest's mount point
     */
    public QuestNavSubsystem(VisionConsumer onMeasurement, Consumer<Pose2d> onReset,
                             Transform3d mountingOffset) {
        super("QuestNav");
        this.onMeasurement  = onMeasurement;
        this.onReset        = onReset;
        this.mountingOffset = mountingOffset;

        // ── Status callbacks ─────────────────────────────────────────────────
        questNav.onConnected(
            () -> System.out.println("[" + getName() + "] Connected to headset."));
        questNav.onDisconnected(
            () -> warn("Headset disconnected — vision paused."));
        questNav.onTrackingLost(
            () -> warn("Tracking lost — check headset view."));
        questNav.onTrackingAcquired(
            () -> System.out.println("[" + getName() + "] Tracking acquired."));
        questNav.onLowBattery(
            20, level -> warn("Headset battery low: " + level + "%"));

        // ── Command confirmation callbacks (QuestNavLib 2026-2.2.0) ──────────
        //
        // These fire inside commandPeriodic() when the Quest headset responds
        // to a setPose command. They drive setKnownPoseCommand's isFinished()
        // check — the command polls poseResetPending, which these callbacks clear.
        //
        // Both fire on the main robot thread so no synchronization is needed.
        questNav.onCommandSuccess(response -> {
            poseResetPending   = false;
            poseResetSucceeded = true;
        });
        questNav.onCommandFailure(response -> {
            poseResetPending   = false;
            poseResetSucceeded = false;
        });
    }

    // =========================================================================
    // Periodic — runs every robot loop
    // =========================================================================

    @Override
    public void periodic() {
        // Must run every loop to process Quest command responses and fire callbacks
        questNav.commandPeriodic();

        // ── Status telemetry ─────────────────────────────────────────────────
        log("Connected",        questNav.isConnected());
        log("Tracking",         questNav.isTracking());
        log("PoseResetPending", poseResetPending);
        questNav.getBatteryPercent().ifPresent(b -> log("Battery", (double) b));

        // Skip vision updates if the Quest isn't reliably tracking
        if (!questNav.isConnected() || !questNav.isTracking()) return;

        // Mark that at least one valid frame has arrived — needed before we
        // can safely send pose commands (see hasSeenFreshFrame guard below)
        hasSeenFreshFrame = true;

        // ── Feed all new frames into the pose estimator ──────────────────────
        // The Quest publishes at 100 Hz; multiple frames can arrive in one loop.
        // Processing all of them keeps the estimator as accurate as possible.
        for (PoseFrame frame : questNav.getAllUnreadPoseFrames()) {
            Pose3d robotPose3d = questPoseToRobotPose(frame.questPose3d());
            Pose2d robotPose2d = robotPose3d.toPose2d();

            onMeasurement.addMeasurement(
                robotPose2d,
                frame.dataTimestamp(),
                VecBuilder.fill(STD_DEVS[0], STD_DEVS[1], STD_DEVS[2])
            );

            log("Pose", robotPose2d);
        }
    }

    // =========================================================================
    // Commands
    // =========================================================================

    /**
     * Sets the robot to a known field pose and waits for Quest to confirm.
     *
     * <p>This replaces the 2026 "mash the button" workflow. The command blocks
     * until the Quest headset acknowledges the reset, then snaps odometry to
     * match. If Quest is disconnected, falls back to an odometry-only reset.
     *
     * <p>Returns {@link CommandBuilder} so callers can chain name and dashboard
     * registration at the call site:
     * <pre>
     *   questNav.setKnownPoseCommand(Nodes.Robot.Start.LEFT)
     *       .named("Set Pose Left")
     *       .publishToDashboard();
     * </pre>
     *
     * @param robotPose the known field pose to lock in (meters, WPILib coordinates)
     */
    public CommandBuilder setKnownPoseCommand(Pose2d robotPose) {
        return new CommandBuilder(this)
            .onInitialize(() -> {
                if (!questNav.isConnected()) {
                    // Quest is offline — reset odometry only and finish immediately.
                    // poseResetSucceeded = true so onEnd snaps odometry.
                    onReset.accept(robotPose);
                    poseResetPending   = false;
                    poseResetSucceeded = true;
                    warn("Not connected — swerve-only pose reset applied.");
                    return;
                }

                if (!hasSeenFreshFrame) {
                    // Quest is connected but hasn't sent a frame yet.
                    // The command will wait — poseResetPending stays true until
                    // the Quest responds. Warn the driver so they know to wait.
                    warn("Waiting for first tracking frame before sending pose reset.");
                }

                poseResetPending   = true;
                poseResetSucceeded = false;

                // Send the Quest's pose (robot center → Quest mount point).
                // The Quest needs its OWN position, not the robot center.
                questNav.setPose(new Pose3d(robotPose).transformBy(mountingOffset));
            })
            // Wait for onCommandSuccess or onCommandFailure to clear poseResetPending.
            // The timeout kicks in if the Quest never responds (disconnected mid-command).
            .isFinished(() -> !poseResetPending)
            .onEnd((interrupted, timedOut, stalled) -> {
                poseResetPending = false;

                if (poseResetSucceeded) {
                    // Snap odometry only after the Quest confirms — prevents a
                    // partially-accepted command from leaving Quest and odometry
                    // in different states.
                    onReset.accept(robotPose);
                    System.out.printf("[%s] Pose locked → x=%.2f m, y=%.2f m, heading=%.1f°%n",
                        getName(), robotPose.getX(), robotPose.getY(),
                        robotPose.getRotation().getDegrees());
                    log("LastPoseReset", robotPose);

                } else if (timedOut) {
                    warn("Pose reset timed out — Quest may not be tracking yet. "
                        + "Check headset connection and tracking state.");

                } else if (interrupted) {
                    warn("Pose reset interrupted before Quest confirmed.");
                }
            })
            .timeout(SET_POSE_TIMEOUT_SECONDS)
            .runsWhenDisabled(true);
    }

    /**
     * Snaps odometry to the Quest's currently reported pose.
     *
     * <p>Use mid-match when QuestNav's accumulated visual tracking is more
     * trustworthy than wheel odometry (e.g. after a collision or a wheel slip).
     * The Quest must already be tracking in field coordinates for this to be valid.
     *
     * <p>This is a one-shot command — it reads the latest frame and finishes
     * immediately. Returns {@link CommandBuilder} for chaining.
     */
    public CommandBuilder resetPoseFromQuestCommand() {
        return new CommandBuilder(this)
            .named("QuestNav.resetFromQuest")
            .onInitialize(() -> {
                if (!questNav.isConnected()) {
                    warn("Cannot reset pose — Quest not connected.");
                    return;
                }

                PoseFrame[] frames = questNav.getAllUnreadPoseFrames();
                if (frames.length == 0) {
                    warn("Cannot reset pose — no frame data available.");
                    return;
                }

                PoseFrame latest = frames[frames.length - 1];
                Pose2d robotPose2d = questPoseToRobotPose(latest.questPose3d()).toPose2d();

                // Snap odometry to what the Quest currently reports
                onReset.accept(robotPose2d);

                // Re-anchor Quest to field coordinates using the same frame's pose.
                // Using the raw questPose3d directly avoids a double-transform.
                questNav.setPose(latest.questPose3d());

                System.out.printf("[%s] Pose reset from Quest → x=%.2f m, y=%.2f m, heading=%.1f°%n",
                    getName(), robotPose2d.getX(), robotPose2d.getY(),
                    robotPose2d.getRotation().getDegrees());
            })
            .isFinished(true);
    }

    // =========================================================================
    // Accessors
    // =========================================================================

    /** Returns true when the Quest headset is connected. */
    public boolean isConnected() {
        return questNav.isConnected();
    }

    /** Returns true when the Quest is actively tracking its position. */
    public boolean isTracking() {
        return questNav.isTracking();
    }

    // =========================================================================
    // Internal helpers
    // =========================================================================

    /**
     * Converts the Quest's reported pose (at its mounting point) to the
     * robot's center pose using the inverse of the mounting offset.
     *
     * Transform directions:
     *   Quest pose → Robot pose: apply mountingOffset.inverse()
     *   Robot pose → Quest pose: apply mountingOffset (forward)
     */
    private Pose3d questPoseToRobotPose(Pose3d questPose) {
        return questPose.transformBy(mountingOffset.inverse());
    }
}
