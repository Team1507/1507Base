//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.swerve;

import java.util.function.Supplier;

import org.wpilib.math.kinematics.ChassisVelocities;

/**
 * Limits how fast a swerve drivetrain speeds up, slows down and starts turning.
 * Swerve.drive() runs every command through it, so teleop and autos both get it.
 *
 * <p>Why: a drivetrain told to go from 0 to full speed instantly
 * <ul>
 *   <li>pulls its highest current at exactly that moment (the main brownout
 *       risk from driving), and</li>
 *   <li>spins the wheels on the carpet, which also throws off odometry.</li>
 * </ul>
 *
 * <p>Three limits, applied in this order each loop (same approach as Team
 * 340's 2026 swerve library, on the same MK5n hardware):
 * <ol>
 *   <li><b>Slip</b> (m/s²): the change in velocity, in any direction, is capped
 *       so the wheels keep grip. Applies to speeding up, slowing down and
 *       changing direction.</li>
 *   <li><b>Torque</b> (m/s² from standstill): speeding up is capped by what the
 *       motors can deliver. A motor has the most torque at a standstill and none
 *       at free speed, so the cap shrinks as the robot goes faster. Only limits
 *       speeding up.</li>
 *   <li><b>Angular</b> (rad/s²): the change in turning rate is capped.</li>
 * </ol>
 *
 * <p>Works on FIELD-relative velocity. On robot-relative velocity, spinning while
 * driving straight would look like acceleration (the robot's axes rotate) and
 * would slow the robot down for no reason.
 *
 * <p>Each loop starts from the previous loop's limited target. If drive() wasn't
 * called recently (the robot was stopped, or another command used stop()), it
 * starts from the robot's measured velocity instead.
 */
public final class SwerveAccelLimiter {

    /**
     * @param slipAccel    maximum change in velocity before the wheels slip (m/s²)
     * @param torqueAccel  maximum acceleration from a standstill that the motors can deliver (m/s²)
     * @param angularAccel maximum change in turning rate (rad/s²)
     * @param maxSpeed     top speed (m/s); the torque limit reaches zero here
     */
    public record Limits(double slipAccel, double torqueAccel, double angularAccel, double maxSpeed) {}

    /** How much the limiter may change the command before it counts as "limiting" (log only). */
    private static final double EPSILON = 1e-6;

    private final Limits limits;
    private final double loopSeconds;

    private double lastVx = 0.0;
    private double lastVy = 0.0;
    private double lastOmega = 0.0;
    private double lastTime = Double.NEGATIVE_INFINITY;
    private boolean limited = false;

    /**
     * @param limits      the three limits and top speed
     * @param loopSeconds how often limit() is called (the robot loop, 0.02 s)
     */
    public SwerveAccelLimiter(Limits limits, double loopSeconds) {
        this.limits = limits;
        this.loopSeconds = loopSeconds;
    }

    /**
     * Returns {@code target}, limited so it changes no faster than the limits
     * allow since the last call.
     *
     * @param target   the velocity a command wants (field-relative)
     * @param measured the robot's measured velocity (field-relative); only read
     *                 when limit() wasn't called in the last few loops
     * @param now      current time in seconds
     */
    public ChassisVelocities limit(ChassisVelocities target, Supplier<ChassisVelocities> measured, double now) {
        // Start from last loop's target, or from what the robot is actually doing.
        if (now - lastTime > loopSeconds * 4.0) {
            ChassisVelocities m = measured.get();
            lastVx = m.vx;
            lastVy = m.vy;
            lastOmega = m.omega;
        }

        double vx = target.vx;
        double vy = target.vy;
        double omega = target.omega;

        // 1. Slip: cap the size of the velocity change, in any direction.
        double dx = vx - lastVx;
        double dy = vy - lastVy;
        double maxChange = limits.slipAccel() * loopSeconds;
        double change = Math.hypot(dx, dy);
        if (change > maxChange) {
            double k = maxChange / change;
            vx = lastVx + k * dx;
            vy = lastVy + k * dy;
        }

        // 2. Torque: cap speeding up; less is available the faster the robot goes.
        double lastSpeed = Math.hypot(lastVx, lastVy);
        double speed = Math.hypot(vx, vy);
        double available = Math.max(0.0,
            limits.torqueAccel() * loopSeconds * (1.0 - lastSpeed / limits.maxSpeed()));
        if (speed - lastSpeed > available && speed > EPSILON) {
            double k = (lastSpeed + available) / speed;
            vx *= k;
            vy *= k;
        }

        // 3. Angular: cap the change in turning rate.
        double maxTurnChange = limits.angularAccel() * loopSeconds;
        double dOmega = omega - lastOmega;
        if (Math.abs(dOmega) > maxTurnChange) {
            omega = lastOmega + Math.copySign(maxTurnChange, dOmega);
        }

        limited = Math.abs(vx - target.vx) > EPSILON
            || Math.abs(vy - target.vy) > EPSILON
            || Math.abs(omega - target.omega) > EPSILON;

        lastVx = vx;
        lastVy = vy;
        lastOmega = omega;
        lastTime = now;
        return new ChassisVelocities(vx, vy, omega);
    }

    /** True if the last limit() call changed the command. */
    public boolean wasLimited() {
        return limited;
    }

    /**
     * Forgets the last target, so the next limit() starts from the measured
     * velocity. Call when the drivetrain is stopped without going through limit().
     */
    public void reset() {
        lastTime = Double.NEGATIVE_INFINITY;
        limited = false;
    }
}
