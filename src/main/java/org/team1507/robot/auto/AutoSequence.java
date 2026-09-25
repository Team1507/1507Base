//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import static org.wpilib.units.Units.Seconds;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.Function;

import org.wpilib.command3.Command;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.system.Timer;
import org.wpilib.telemetry.Telemetry;

import org.team1507.lib.core.auto.ClassicRouteDriver;
import org.team1507.lib.core.auto.RouteConfig;
import org.team1507.lib.core.auto.RouteDriver;
import org.team1507.lib.core.auto.RouteGeometry;
import org.team1507.lib.core.auto.RouteNode;
import org.team1507.lib.core.auto.RouteProgress;
import org.team1507.lib.core.auto.RouteRunner;
import org.team1507.lib.core.util.Alliance;
import org.team1507.robot.Constants.kAuto;
import org.team1507.robot.Constants.kAuto.Accuracy;
import org.team1507.robot.auto.nodes.FieldFlip;
import org.team1507.robot.auto.nodes.Nodes;
import org.team1507.robot.subsystems.Swerve;
import org.team1507.robot.subsystems.SwerveConfig;

import static org.team1507.robot.Constants.kSwerve.kTuning.*;

// ─────────────────────────────────────────────────────────────────────────────
// AutoSequence
//
// Builds an autonomous routine as a list of steps, top to bottom, one line per
// thing that happens. build() turns the list into the command the auto runs.
//
//   new AutoSequence()                       // classic driver (the default)
//       .resetPose(Nodes.Robot.Start.RIGHT)
//       .checkpoint(OVER_BUMP, Accuracy.TIGHT)
//       .intakeDeploy()                      // happens while the robot drives on
//       .slow().checkpoint(SUBWAY_ENTRY)
//       .intakeRetract()                     // happens while the robot drives on
//       .endpoint(SHOOT_SPOT).facing(Nodes.FieldElements.Hub.CENTER)
//       .shootUntil(19.99)                   // happens while stopped
//       .build();
//
// PATH STEPS
//   .checkpoint(node)   drive THROUGH it. The list waits until the robot gets
//                       there, then the next steps run WHILE THE ROBOT KEEPS
//                       DRIVING toward the next endpoint.
//   .waypoint(node)     a shaping point: keeps the route where you want it.
//                       Not a step; nothing waits for it.
//   .endpoint(node)     drive to it and STOP. The next steps run while stopped.
//
//   THE ONE RULE: after a checkpoint, the robot keeps driving until the next
//   endpoint. To stop and then do something, use an endpoint.
//
//   How it works: at the first path step, the robot starts driving the whole
//   route up to the next endpoint in the background. Path steps only wait for
//   "reached". If an action takes long enough that the robot passes the next
//   checkpoint first, that checkpoint's wait finishes at once; nothing is missed.
//
// ACCURACY (checkpoints and waypoints)
//   .checkpoint(node)                        NORMAL
//   .checkpoint(node, Accuracy.PRECISE)      a preset (distance AND heading), from Constants.kAuto
//   .checkpoint(node, 0.3, 20)               exact: 0.3 m and 20°, for rare cases
//
// HEADING (right after a path step)
//   The robot always turns toward the node's heading (Node.at(x, y, degrees)).
//   .endpoint(node)                          finishes on position only (fastest)
//   .endpoint(node).heading()                also waits for the node's heading
//   .endpoint(node).heading(90)              turns to 90° instead, and waits for it
//   .endpoint(node).facing(Hub.CENTER)       faces a field location, and waits for it
//   On checkpoints, .heading(deg) / .facing(...) only change which way the robot
//   turns; the checkpoint's accuracy decides how close the heading must be.
//
// TIME CUTOFF (right after a path step)
//   .checkpoint(node).by(2.7)                if not reached by 2.7 s into auto,
//                                            count it as reached and move on
//
// SPEED
//   new AutoSequence().maxSpeed(0.8)         the whole auto at 80% of top speed
//   .slow().checkpoint(node)                 half of the auto's speed, this leg only
//   .creep().checkpoint(node)                30% of the auto's speed, this leg only
//   .withSpeed(2.0).checkpoint(node)         2.0 m/s, this leg only
//   A speed modifier must come right before a path step (or driveForwardMeters).
//
// LEFT / RIGHT MIRRORING
//   Write routines for the RIGHT side of the field (as seen from your driver
//   station). When the auto is enabled, it checks where the robot is (QuestNav,
//   or the dashboard seed buttons, see PoseSeeds). On the LEFT side, every node
//   is mirrored. If nothing has told the robot where it is, it runs as written.
//   .side(Side.LEFT)    always run mirrored          .noMirror()    never mirror
//   (use .noMirror() for routines written for one specific side, or center starts)
//   Red flipping happens automatically on top of this.
//
// DRIVERS
//   new AutoSequence()                classic: straight lines between nodes. Add
//                                     nodes to steer around obstacles; the build
//                                     checks for legs that pass too close to one.
//   new AutoSequence(Driver.POLICY)   the trained policy. It steers around
//                                     obstacles itself; nodes at most 5 m apart.
//
// MISTAKES ARE CAUGHT BEFORE THE ROBOT MOVES
//   build() refuses a routine with a misplaced modifier, a drivetrain step while
//   the robot is still driving a route, and similar mistakes, with a message
//   saying what to fix. The AutoRoutinesTest builds every routine on every build.
//
// HOW TO ADD NEW AUTO STEPS (each year)
//   1. Write the command on your subsystem (or in RobotBehaviors.java if it uses
//      several subsystems). Subsystems are reachable as AutoBuilder.robot.<name>.
//   2. Add a one-line wrapper method in the ROBOT ACTIONS section below.
//   3. Use it in your routine file.
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoSequence {

    // =========================================================================
    // What the list holds
    // =========================================================================

    /** Something the auto does, in order. Created as data; build() makes the commands. */
    private sealed interface Step permits PathStep, RunStep, BackgroundStep {}

    /** Wait until route node {@code node} is done (starting that part of the route if needed). */
    private record PathStep(int node) implements Step {}

    /** Run a command and wait for it to finish. */
    private record RunStep(Function<Placement, Command> make) implements Step {}

    /** Start a command and move on at once (keep-running wrappers). */
    private record BackgroundStep(Function<Placement, Command> make) implements Step {}

    /** A route node as written in the routine (Blue, right side); placed on the field by build(). */
    private static final class NodeSpec {
        RouteNode.Type type;
        Pose2d node;
        double meters;
        double degrees;
        boolean checksHeading;
        double headingDeg = Double.NaN;          // .heading(deg) override
        Translation2d facing = null;              // .facing(location)
        double speedMps = Double.NaN;             // .withSpeed(...)
        double speedFraction = Double.NaN;        // .slow() / .creep()
        double cutoffSeconds = Double.POSITIVE_INFINITY;
    }

    /** Where the routine runs: mirrored to the left side, and/or flipped for Red. */
    private record Placement(boolean mirror, boolean red) {
        Pose2d pose(Pose2d p) {
            if (mirror) p = FieldFlip.mirror(p);
            return red ? FieldFlip.pose(p) : p;
        }
        Translation2d translation(Translation2d t) {
            if (mirror) t = FieldFlip.mirror(t);
            return red ? FieldFlip.translation(t) : t;
        }
        Rotation2d heading(Rotation2d r) {
            if (mirror) r = FieldFlip.mirror(r);
            return red ? FieldFlip.rotation(r) : r;
        }
    }

    // =========================================================================
    // Fields
    // =========================================================================

    private final Driver driver;
    private final boolean isBranch;
    private final List<Step> steps = new ArrayList<>();
    private final List<String> stepNames = new ArrayList<>();
    private final List<NodeSpec> nodes = new ArrayList<>();
    private final List<String> mistakes = new ArrayList<>();

    // Autonomous timer: restarted when the routine starts, read by .by(),
    // .waitUntilTime() and shootUntil()-style wrappers. Shared with branches.
    private final Timer autoTimer;

    private double maxSpeedFraction = 1.0;
    private Side forcedSide = null;
    private boolean noMirror = false;

    // Speed modifier waiting for the next path step (or driveForwardMeters).
    private double pendingSpeedMps = Double.NaN;
    private double pendingSpeedFraction = Double.NaN;
    private String pendingSpeedName = null;

    /** The node just added, for .heading() / .facing() / .by(); null after any other step. */
    private NodeSpec lastNode = null;

    /** Where resetPose put the robot (Blue, right side), for the route checks. */
    private Pose2d startPose = null;

    /** Warnings from the most recent build(), for AutoRoutinesTest. */
    private static List<String> lastWarnings = List.of();

    // =========================================================================
    // Constructors
    // =========================================================================

    /** A routine driven by the classic driver (straight lines between nodes). */
    public AutoSequence() {
        this(Driver.CLASSIC);
    }

    /** A routine driven by the given driver: {@code new AutoSequence(Driver.POLICY)}. */
    public AutoSequence(Driver driver) {
        this.driver = driver;
        this.isBranch = false;
        this.autoTimer = new Timer();
    }

    /** A branch inside parallel/race/deadline: shares the timer; actions only. */
    private AutoSequence(AutoSequence parent) {
        this.driver = parent.driver;
        this.isBranch = true;
        this.autoTimer = parent.autoTimer;
    }

    // =========================================================================
    // WHOLE-ROUTINE SETTINGS
    // =========================================================================

    /** Caps the whole auto's speed as a fraction of top speed, e.g. 0.8. */
    public AutoSequence maxSpeed(double fraction) {
        this.maxSpeedFraction = fraction;
        return this;
    }

    /** Always runs this routine on the given side (LEFT = mirrored). */
    public AutoSequence side(Side side) {
        this.forcedSide = side;
        return this;
    }

    /** Never mirrors this routine: for routines written for one specific side, or center starts. */
    public AutoSequence noMirror() {
        this.noMirror = true;
        return this;
    }

    // =========================================================================
    // PATH STEPS
    // =========================================================================

    /** Drive through this node (NORMAL accuracy); the next steps run while driving on. */
    public AutoSequence checkpoint(Pose2d node) {
        return checkpoint(node, Accuracy.NORMAL);
    }

    /** Drive through this node with an accuracy preset. */
    public AutoSequence checkpoint(Pose2d node, Accuracy accuracy) {
        return checkpoint(node, accuracy.meters, accuracy.degrees);
    }

    /** Drive through this node within {@code meters} and {@code degrees}. For rare cases; prefer a preset. */
    public AutoSequence checkpoint(Pose2d node, double meters, double degrees) {
        int index = addNode(RouteNode.Type.CHECKPOINT, node, meters, degrees, true);
        steps.add(new PathStep(index));
        stepNames.add("checkpoint " + index);
        return this;
    }

    /** A shaping point (NORMAL accuracy). Not a step: nothing waits for it. */
    public AutoSequence waypoint(Pose2d node) {
        return waypoint(node, Accuracy.NORMAL);
    }

    /** A shaping point with an accuracy preset. */
    public AutoSequence waypoint(Pose2d node, Accuracy accuracy) {
        addNode(RouteNode.Type.WAYPOINT, node, accuracy.meters, accuracy.degrees, true);
        return this;
    }

    /** Drive to this node and stop. Finishes on position; add .heading() or .facing() to also wait for heading. */
    public AutoSequence endpoint(Pose2d node) {
        int index = addNode(RouteNode.Type.ENDPOINT, node, ARRIVE_THRESHOLD, HEADING_TOLERANCE_DEG, false);
        steps.add(new PathStep(index));
        stepNames.add("endpoint " + index);
        return this;
    }

    // ---- Modifiers for the path step just added ----

    /** The endpoint also waits until the robot reaches the node's heading. */
    public AutoSequence heading() {
        requireLastNode("heading()").checksHeading = true;
        return this;
    }

    /** Turns to {@code degrees} instead of the node's heading (and an endpoint waits for it). */
    public AutoSequence heading(double degrees) {
        NodeSpec node = requireLastNode("heading(" + degrees + ")");
        node.headingDeg = degrees;
        node.checksHeading = true;
        return this;
    }

    /** Faces a field location from this node (and an endpoint waits for it), e.g. .facing(Hub.CENTER). */
    public AutoSequence facing(Translation2d location) {
        NodeSpec node = requireLastNode("facing(...)");
        node.facing = location;
        node.checksHeading = true;
        return this;
    }

    /** Faces a field location given as a pose (its heading is ignored). */
    public AutoSequence facing(Pose2d location) {
        return facing(location.getTranslation());
    }

    /** If this node isn't reached by {@code autoSeconds} into auto, count it as reached and move on. */
    public AutoSequence by(double autoSeconds) {
        requireLastNode("by(" + autoSeconds + ")").cutoffSeconds = autoSeconds;
        return this;
    }

    // =========================================================================
    // SPEED MODIFIERS (right before a path step or driveForwardMeters)
    // =========================================================================

    /** This leg at {@code metersPerSecond} (capped by maxSpeed). */
    public AutoSequence withSpeed(double metersPerSecond) {
        setPendingSpeed(metersPerSecond, Double.NaN, "withSpeed(" + metersPerSecond + ")");
        return this;
    }

    /** This leg at half of the auto's speed. */
    public AutoSequence slow() {
        setPendingSpeed(Double.NaN, 0.5, "slow()");
        return this;
    }

    /** This leg at 30% of the auto's speed: final alignment, tight spots. */
    public AutoSequence creep() {
        setPendingSpeed(Double.NaN, 0.3, "creep()");
        return this;
    }

    // =========================================================================
    // DRIVETRAIN STEPS (not while a route is still driving)
    // =========================================================================

    /** Sets the robot's pose to where it was placed. Put it first. Mirrored/flipped like every node. */
    public AutoSequence resetPose(Pose2d pose) {
        startPose = pose;
        return add("resetPose", place -> swerve().resetPoseCommand(place.pose(pose)));
    }

    /** Drives forward {@code meters} along the current heading and stops (e.g. creep into a wall). */
    public AutoSequence driveForwardMeters(double meters) {
        return driveForwardMeters(meters, true);
    }

    /** Drives forward {@code meters}; {@code stopAtEnd = false} leaves the wheels moving. */
    public AutoSequence driveForwardMeters(double meters, boolean stopAtEnd) {
        double mps = pendingSpeedMps;
        double fraction = pendingSpeedFraction;
        clearPendingSpeed();
        return add("driveForwardMeters",
            place -> swerve().driveForwardMeters(meters, legSpeed(mps, fraction), stopAtEnd));
    }

    /** Turns in place to face a field location. (At an endpoint, prefer .facing(...).) */
    public AutoSequence pointToTarget(Translation2d location) {
        return add("pointToTarget", place -> swerve().pointToTarget(new Pose2d(place.translation(location), Rotation2d.ZERO)));
    }

    /** Turns in place to face a field location given as a pose (its heading is ignored). */
    public AutoSequence pointToTarget(Pose2d location) {
        return pointToTarget(location.getTranslation());
    }

    /** Turns in place to a heading in degrees (mirrored/flipped like every node). */
    public AutoSequence changeHeading(double degrees) {
        return add("changeHeading", place -> swerve().changeHeading(place.heading(Rotation2d.fromDegrees(degrees))));
    }

    /** Turns in place to the heading stored in a node. */
    public AutoSequence changeHeading(Pose2d node) {
        return add("changeHeading", place -> swerve().changeHeading(place.pose(node).getRotation()));
    }

    /** Stops the drivetrain. */
    public AutoSequence stop() {
        return add("stop", place -> swerve().stopCommand());
    }


    // =========================================================================
    // ROBOT ACTIONS
    //
    // One-line wrappers around subsystem commands, so routines read like a list.
    // Subsystems are reachable as AutoBuilder.robot.<name>.
    //
    // An action that FINISHES (deploy an arm, shoot until a time):
    //   public AutoSequence intakeDeploy() {
    //       return add("intakeDeploy", place -> AutoBuilder.robot.intake.deployCommand());
    //   }
    //
    // An action that KEEPS RUNNING (rollers): start it in the background, so the
    // list moves on at once. A later step for the same subsystem stops it (e.g.
    // intakeRollersOff()), and so does the end of the auto.
    //   public AutoSequence intakeRollersOn() {
    //       return runInBackground("intakeRollersOn", AutoBuilder.robot.intake.runRollersCommand());
    //   }
    //
    // (Why: when a command finishes, the subsystem's default command, idle(),
    // takes over and stops the motor. So "rollers on" must keep running, and a
    // step that never finishes would block every step after it.)
    //
    // A multi-subsystem behavior from RobotBehaviors:
    //   public AutoSequence ejectPiece() {
    //       return add("ejectPiece", place -> RobotBehaviors.ejectPiece());
    //   }
    // =========================================================================


    // =========================================================================
    // GROUPS: PARALLEL / RACE / DEADLINE (rarely needed)
    //
    // Checkpoints cover "do X while driving". Groups are for running several
    // actions together. Each branch is its own little list, written as a lambda:
    //     seq -> seq.step1().step2()
    // Branches can't contain path steps (checkpoint / waypoint / endpoint).
    // =========================================================================

    /** Runs all branches at once; ends when ALL are done. */
    public AutoSequence parallel(Branch... branches) {
        List<AutoSequence> subs = branches(branches);
        return add("parallel", place -> Command.parallel(commands(subs, place)).named("parallel"));
    }

    /** Runs all branches at once; ends when the FIRST finishes, canceling the rest. */
    public AutoSequence race(Branch... branches) {
        List<AutoSequence> subs = branches(branches);
        return add("race", place -> Command.race(commands(subs, place)).named("race"));
    }

    /** Runs all branches at once; ends when the FIRST branch (the deadline) finishes. */
    public AutoSequence deadline(Branch deadlineBranch, Branch... others) {
        AutoSequence deadline = branches(deadlineBranch).get(0);
        List<AutoSequence> rest = branches(others);
        // v3: the deadline is the only REQUIRED command; the group ends when it
        // finishes and cancels the OPTIONAL others.
        return add("deadline", place -> Command.parallel(deadline.buildBranch(place))
            .optional(commands(rest, place))
            .named("deadline"));
    }

    // =========================================================================
    // TIME AND WAITING
    // =========================================================================

    /** Restarts the auto timer. Rarely needed: it starts automatically with the routine. */
    public AutoSequence startTimer() {
        return add("startTimer", place -> Command.noRequirements(co -> autoTimer.restart()).named("startTimer"));
    }

    /** Waits until the auto timer reaches {@code autoSeconds}. */
    public AutoSequence waitUntilTime(double autoSeconds) {
        return add("waitUntilTime", place -> timerReaches(autoSeconds));
    }

    /** A command that finishes when the auto timer reaches {@code autoSeconds}, for wrappers like shootUntil. */
    public Command endAtTime(double autoSeconds) {
        return timerReaches(autoSeconds);
    }

    /** Waits a fixed number of seconds. */
    public AutoSequence waitSeconds(double seconds) {
        return add("waitSeconds", place -> Command.waitFor(Seconds.of(seconds)).named("wait " + seconds + "s"));
    }

    /** Waits until a condition is true, e.g. .waitUntil(AutoBuilder.robot.intake::hasPiece). */
    public AutoSequence waitUntil(BooleanSupplier condition) {
        return add("waitUntil", place -> Command.waitUntil(condition).named("waitUntil"));
    }

    /** Adds any command as a step (waits for it to finish). */
    public AutoSequence addCommand(Command command) {
        return add(command.name(), place -> command);
    }

    /** Renames the step just added; the name shows in the command log (Commands/Events). */
    public AutoSequence withName(String name) {
        if (steps.isEmpty() || !(steps.get(steps.size() - 1) instanceof RunStep last)) {
            mistakes.add("withName(\"" + name + "\") must come right after an action step");
            return this;
        }
        steps.set(steps.size() - 1, new RunStep(place -> {
            Command step = last.make().apply(place);
            return Command.requiring(step.requirements())
                .executing(coroutine -> coroutine.await(step))
                .named(name);
        }));
        stepNames.set(stepNames.size() - 1, name);
        return this;
    }

    // =========================================================================
    // BUILD
    // =========================================================================

    /**
     * Builds the command the auto runs. Call it at the end of every routine's
     * build() method. Throws with a list of mistakes to fix if the routine
     * isn't valid.
     */
    public Command build() {
        if (isBranch) {
            throw new IllegalStateException("build() is for the whole routine, not a branch");
        }
        if (!Double.isNaN(pendingSpeedMps) || !Double.isNaN(pendingSpeedFraction)) {
            mistakes.add(pendingSpeedName + " at the end of the routine does nothing: "
                + "put it right before a checkpoint, waypoint or endpoint");
        }
        for (int i = nodes.size() - 1; i >= 0 && nodes.get(i).type == RouteNode.Type.WAYPOINT; i--) {
            mistakes.add("waypoint " + i + " comes after the last checkpoint/endpoint, so the robot "
                + "would never drive to it: end the route with a checkpoint or endpoint");
        }

        Placement place = placement();
        List<RouteNode> route = placeNodes(place);
        int[] partFirst = new int[route.size()];
        int[] partLast = new int[route.size()];
        for (int i = 0, first = 0; i < route.size(); i++) {
            partFirst[i] = first;
            if (route.get(i).type() == RouteNode.Type.ENDPOINT || i == route.size() - 1) {
                for (int j = first; j <= i; j++) partLast[j] = i;
                first = i + 1;
            }
        }

        // Make every step's command now: catches mistakes before the robot
        // moves, and warms up the code so the first loop of auto isn't slow.
        List<Command> made = new ArrayList<>();
        for (Step step : steps) {
            made.add(switch (step) {
                case RunStep r -> r.make().apply(place);
                case BackgroundStep b -> b.make().apply(place);
                case PathStep p -> null;
            });
        }
        checkDrivetrainSteps(made, partLast);

        RouteDriver routeDriver = routeDriver(!route.isEmpty());
        if (!mistakes.isEmpty()) {
            throw new IllegalStateException("This auto routine has mistakes to fix:\n  - "
                + String.join("\n  - ", mistakes));
        }

        List<String> warnings = checkRoute();
        lastWarnings = List.copyOf(warnings);
        for (String warning : warnings) {
            DriverStationErrors.reportWarning("Auto route: " + warning, false);
        }

        RouteProgress progress = new RouteProgress();
        RouteConfig config = new RouteConfig(HEADING_KP, ARRIVE_KP, kAuto.ENDPOINT_HANDOFF,
            STALL_THRESHOLD, STALL_TIMEOUT, kAuto.MAX_SECONDS_PER_NODE, kAuto.HEADING_WAIT_SECONDS);
        Command[] parts = new Command[route.size()];
        for (int i = 0; i < route.size(); i++) {
            if (partFirst[i] == i) {
                parts[i] = RouteRunner.drive(swerve(), route, i, partLast[i], routeDriver, progress,
                    config, autoTimer::get);
            }
        }

        Telemetry.log("Auto/Mirrored", place.mirror());
        Telemetry.log("Auto/Driver", driver.name());

        return Command.noRequirements(coroutine -> {
            autoTimer.restart();
            boolean[] started = new boolean[route.size()];
            for (int i = 0; i < steps.size(); i++) {
                switch (steps.get(i)) {
                    case PathStep p -> {
                        int first = partFirst[p.node()];
                        if (!started[first]) {
                            started[first] = true;
                            coroutine.fork(parts[first]);   // drive in the background
                        }
                        coroutine.waitUntil(() -> progress.isDone(p.node()));
                    }
                    case RunStep r -> coroutine.await(made.get(i));
                    case BackgroundStep b -> coroutine.fork(made.get(i));
                }
            }
            // Don't end while the robot is still driving: ending would cancel the route.
            if (!route.isEmpty()) {
                coroutine.waitUntil(() -> progress.isDone(route.size() - 1));
            }
        }).named("AutoSequence");
    }

    /** Warnings from the most recent build() (route checks). Empty means the route passed. */
    public static List<String> warningsFromLastBuild() {
        return lastWarnings;
    }

    // =========================================================================
    // Internal
    // =========================================================================

    /**
     * For wrappers: starts a keep-running command in the background (see ROBOT
     * ACTIONS). Not public: routines use the wrapper, never this directly.
     */
    AutoSequence runInBackground(String name, Command command) {
        if (isBranch) {
            mistakes.add(name + " (keeps running) can't go inside parallel/race/deadline");
            return this;
        }
        beforeNonPathStep(name);
        steps.add(new BackgroundStep(place -> command));
        stepNames.add(name);
        return this;
    }

    private AutoSequence add(String name, Function<Placement, Command> make) {
        beforeNonPathStep(name);
        steps.add(new RunStep(make));
        stepNames.add(name);
        return this;
    }

    private void beforeNonPathStep(String name) {
        if (!Double.isNaN(pendingSpeedMps) || !Double.isNaN(pendingSpeedFraction)) {
            mistakes.add(pendingSpeedName + " is right before " + name + "(), so it does nothing: "
                + "put it right before a checkpoint, waypoint, endpoint or driveForwardMeters");
            clearPendingSpeed();
        }
        lastNode = null;
    }

    private int addNode(RouteNode.Type type, Pose2d node, double meters, double degrees, boolean checksHeading) {
        if (isBranch) {
            mistakes.add(type.name().toLowerCase() + " can't go inside parallel/race/deadline: "
                + "use checkpoints and put the actions after them");
        }
        NodeSpec spec = new NodeSpec();
        spec.type = type;
        spec.node = node;
        spec.meters = meters;
        spec.degrees = degrees;
        spec.checksHeading = checksHeading;
        spec.speedMps = pendingSpeedMps;
        spec.speedFraction = pendingSpeedFraction;
        clearPendingSpeed();
        nodes.add(spec);
        lastNode = spec;
        return nodes.size() - 1;
    }

    private NodeSpec requireLastNode(String modifier) {
        if (lastNode == null) {
            mistakes.add(modifier + " must come right after a checkpoint, waypoint or endpoint");
            return new NodeSpec();   // throwaway, so the chain can continue
        }
        return lastNode;
    }

    private void setPendingSpeed(double mps, double fraction, String name) {
        if (pendingSpeedName != null) {
            mistakes.add(pendingSpeedName + " is followed by " + name + ": use one speed modifier per leg");
        }
        pendingSpeedMps = mps;
        pendingSpeedFraction = fraction;
        pendingSpeedName = name;
    }

    private void clearPendingSpeed() {
        pendingSpeedMps = Double.NaN;
        pendingSpeedFraction = Double.NaN;
        pendingSpeedName = null;
    }

    /** Speed for one leg: its modifier (if any), capped by the auto's maxSpeed. */
    private double legSpeed(double mps, double fraction) {
        double autoMax = swerve().getMaxSpeed() * maxSpeedFraction;
        if (!Double.isNaN(mps)) return Math.min(mps, autoMax);
        if (!Double.isNaN(fraction)) return autoMax * fraction;
        return autoMax;
    }

    private Placement placement() {
        boolean mirror;
        if (noMirror) {
            mirror = false;
        } else if (forcedSide != null) {
            mirror = forcedSide == Side.LEFT;
        } else {
            mirror = detectSide() == Side.LEFT;
        }
        return new Placement(mirror, Alliance.isRed());
    }

    /** The side the robot is on, from its pose; RIGHT (as written) if nothing has set the pose. */
    private Side detectSide() {
        Swerve swerve = swerve();
        if (!swerve.hasPoseBeenSet()) {
            return Side.RIGHT;
        }
        Pose2d pose = swerve.getPose();
        if (Alliance.isRed()) {
            pose = FieldFlip.pose(pose);   // back to Blue coordinates (the flip is its own inverse)
        }
        return pose.getY() > Nodes.Field.WIDTH / 2.0 ? Side.LEFT : Side.RIGHT;
    }

    private List<RouteNode> placeNodes(Placement place) {
        List<RouteNode> placed = new ArrayList<>();
        for (NodeSpec spec : nodes) {
            Pose2d target = place.pose(spec.node);
            Rotation2d heading;
            if (spec.facing != null) {
                Translation2d toTarget = place.translation(spec.facing).minus(target.getTranslation());
                heading = toTarget.getAngle().orElse(target.getRotation());
            } else if (!Double.isNaN(spec.headingDeg)) {
                heading = place.heading(Rotation2d.fromDegrees(spec.headingDeg));
            } else {
                heading = target.getRotation();
            }
            placed.add(new RouteNode(spec.type, target, heading, spec.meters, spec.degrees,
                spec.checksHeading, legSpeed(spec.speedMps, spec.speedFraction), spec.cutoffSeconds));
        }
        return placed;
    }

    /** A step that uses the drivetrain while a route is still driving would cancel the route. */
    private void checkDrivetrainSteps(List<Command> made, int[] partLast) {
        Swerve swerve = swerve();
        int drivingUntil = -1;   // last node of the route part still driving, or -1
        for (int i = 0; i < steps.size(); i++) {
            Step step = steps.get(i);
            if (step instanceof PathStep p) {
                drivingUntil = p.node() < partLast[p.node()] ? partLast[p.node()] : -1;
                continue;
            }
            Command command = made.get(i);
            if (step instanceof BackgroundStep && command.requires(swerve)) {
                mistakes.add(stepNames.get(i) + " uses the drivetrain and keeps running: "
                    + "that would fight the route");
            } else if (drivingUntil >= 0 && command.requires(swerve)) {
                mistakes.add(stepNames.get(i) + "() uses the drivetrain while the robot is still "
                    + "driving the route (to node " + drivingUntil + "). Put it after that endpoint, "
                    + "or use .heading()/.facing() on the node instead of a turn step.");
            }
        }
    }

    private RouteDriver routeDriver(boolean hasRoute) {
        if (driver == Driver.CLASSIC) {
            return new ClassicRouteDriver(ARRIVE_KP);
        }
        if (AutoBuilder.policyDriver == null && hasRoute) {
            mistakes.add("Driver.POLICY needs the policy driver, which isn't installed on this robot. "
                + "Pick the non-policy version of this auto.");
        }
        return AutoBuilder.policyDriver;
    }

    /**
     * Route checks on the routine as written (Blue, right side; the field is
     * symmetric). Classic: no straight leg may pass too close to a field element.
     * Policy: nodes at most kAuto.POLICY_MAX_NODE_SPACING apart.
     */
    private List<String> checkRoute() {
        List<String> warnings = new ArrayList<>();
        Translation2d previous = startPose == null ? null : startPose.getTranslation();
        String previousName = "the start pose";
        double clearance = SwerveConfig.DRIVE_BASE_RADIUS + 0.1;
        List<Translation2d[]> obstacles = new ArrayList<>();
        List<String> obstacleNames = new ArrayList<>();
        collectFieldElements(obstacles, obstacleNames);

        for (int i = 0; i < nodes.size(); i++) {
            Translation2d here = nodes.get(i).node.getTranslation();
            String hereName = nodes.get(i).type.name().toLowerCase() + " " + i;
            if (previous != null) {
                if (driver == Driver.POLICY) {
                    double spacing = previous.getDistance(here);
                    if (spacing > kAuto.POLICY_MAX_NODE_SPACING) {
                        warnings.add(String.format("%s is %.1f m from %s; the policy needs nodes at most "
                            + "%.1f m apart (add a waypoint)", hereName, spacing, previousName,
                            kAuto.POLICY_MAX_NODE_SPACING));
                    }
                } else {
                    for (int k = 0; k < obstacles.size(); k++) {
                        double gap = RouteGeometry.clearance(previous, here, obstacles.get(k));
                        if (gap < clearance) {
                            warnings.add(String.format("the straight leg from %s to %s passes %.2f m "
                                + "from %s (the robot needs %.2f m): add a waypoint to go around it",
                                previousName, hereName, gap, obstacleNames.get(k), clearance));
                        }
                    }
                }
            }
            previous = here;
            previousName = hereName;
        }
        return warnings;
    }

    /** Every Nodes.FieldElements class with a CORNERS array. */
    private static void collectFieldElements(List<Translation2d[]> obstacles, List<String> names) {
        for (Class<?> element : Nodes.FieldElements.class.getDeclaredClasses()) {
            try {
                Field corners = element.getDeclaredField("CORNERS");
                if (Modifier.isStatic(corners.getModifiers())
                        && corners.getType() == Translation2d[].class) {
                    obstacles.add((Translation2d[]) corners.get(null));
                    names.add("FieldElements." + element.getSimpleName());
                }
            } catch (ReflectiveOperationException e) {
                // no CORNERS: not an obstacle
            }
        }
    }

    private List<AutoSequence> branches(Branch... branches) {
        List<AutoSequence> subs = new ArrayList<>();
        for (Branch branch : branches) {
            AutoSequence sub = new AutoSequence(this);
            branch.build(sub);
            mistakes.addAll(sub.mistakes);
            subs.add(sub);
        }
        return subs;
    }

    private static Command[] commands(List<AutoSequence> subs, Placement place) {
        return subs.stream().map(sub -> sub.buildBranch(place)).toArray(Command[]::new);
    }

    /** A branch's steps as one sequential command (branches hold actions only). */
    private Command buildBranch(Placement place) {
        List<Command> commands = new ArrayList<>();
        for (Step step : steps) {
            if (step instanceof RunStep r) {
                commands.add(r.make().apply(place));
            }
        }
        if (commands.isEmpty()) {
            return Command.noRequirements(coroutine -> {}).named("empty");
        }
        return Command.sequence(commands.toArray(Command[]::new)).withAutomaticName();
    }

    private Command timerReaches(double autoSeconds) {
        return Command.waitUntil(() -> autoTimer.get() >= autoSeconds)
            .named("until t=" + autoSeconds + "s");
    }

    private static Swerve swerve() {
        return AutoBuilder.robot.swerve;
    }

    // =========================================================================
    // BRANCH INTERFACE (for parallel / race / deadline)
    // =========================================================================

    /** A branch inside a group, written as a lambda: {@code seq -> seq.step1().step2()}. */
    @FunctionalInterface
    public interface Branch {
        void build(AutoSequence seq);
    }
}
