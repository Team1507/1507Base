//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.lib.core.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

import org.wpilib.math.kinematics.ChassisVelocities;

// ─────────────────────────────────────────────────────────────────────────────
// SwerveAccelLimiterTest
//
// Checks the acceleration limits Swerve.drive() applies to every command,
// using Team 340's starting limits and a 5 m/s robot.
// ─────────────────────────────────────────────────────────────────────────────
class SwerveAccelLimiterTest {

    private static final double DT = 0.02;
    private static final SwerveAccelLimiter.Limits LIMITS =
        new SwerveAccelLimiter.Limits(15.0, 10.0, 32.0, 5.0);
    private static final ChassisVelocities STOPPED = new ChassisVelocities();

    private double now = 0.0;

    private ChassisVelocities step(SwerveAccelLimiter limiter, ChassisVelocities target, ChassisVelocities measured) {
        now += DT;
        return limiter.limit(target, () -> measured, now);
    }

    private static double speed(ChassisVelocities v) {
        return Math.hypot(v.vx, v.vy);
    }

    @Test
    void fullStickFromAStopRampsUpOverSeveralLoops() {
        SwerveAccelLimiter limiter = new SwerveAccelLimiter(LIMITS, DT);
        ChassisVelocities fullSpeed = new ChassisVelocities(5.0, 0.0, 0.0);

        ChassisVelocities first = step(limiter, fullSpeed, STOPPED);
        assertTrue(limiter.wasLimited());
        // From a standstill the torque limit allows 10 m/s² × 0.02 s = 0.2 m/s.
        assertEquals(0.2, speed(first), 1e-9);

        int loops = 1;
        ChassisVelocities v = first;
        while (speed(v) < 0.95 * 5.0 && loops < 500) {
            v = step(limiter, fullSpeed, STOPPED);
            loops++;
        }
        // Torque falls off with speed, so reaching 95% takes about 1.5 s.
        assertTrue(loops > 25 && loops < 200, "loops to 95% speed: " + loops);
    }

    @Test
    void brakingIsLimitedByWheelSlip() {
        SwerveAccelLimiter limiter = new SwerveAccelLimiter(LIMITS, DT);
        ChassisVelocities moving = new ChassisVelocities(4.0, 0.0, 0.0);

        // First call after a pause starts from the measured velocity (4 m/s).
        ChassisVelocities v = step(limiter, STOPPED, moving);
        // Slip allows 15 m/s² × 0.02 s = 0.3 m/s of change per loop.
        assertEquals(3.7, v.vx, 1e-9);
        assertTrue(limiter.wasLimited());
    }

    @Test
    void changingDirectionIsLimitedByWheelSlip() {
        SwerveAccelLimiter limiter = new SwerveAccelLimiter(LIMITS, DT);
        ChassisVelocities forward = new ChassisVelocities(3.0, 0.0, 0.0);
        ChassisVelocities left = new ChassisVelocities(0.0, 3.0, 0.0);

        ChassisVelocities v = step(limiter, left, forward);
        double change = Math.hypot(v.vx - forward.vx, v.vy - forward.vy);
        assertEquals(0.3, change, 1e-9);
    }

    @Test
    void turningRateIsLimited() {
        SwerveAccelLimiter limiter = new SwerveAccelLimiter(LIMITS, DT);
        ChassisVelocities spin = new ChassisVelocities(0.0, 0.0, 10.0);

        ChassisVelocities v = step(limiter, spin, STOPPED);
        // 32 rad/s² × 0.02 s = 0.64 rad/s per loop.
        assertEquals(0.64, v.omega, 1e-9);
    }

    @Test
    void steadyDrivingIsNotLimited() {
        SwerveAccelLimiter limiter = new SwerveAccelLimiter(LIMITS, DT);
        ChassisVelocities cruising = new ChassisVelocities(3.0, 1.0, 1.0);

        ChassisVelocities v = step(limiter, cruising, cruising);
        assertFalse(limiter.wasLimited());
        assertEquals(3.0, v.vx, 1e-9);
        assertEquals(1.0, v.vy, 1e-9);
        assertEquals(1.0, v.omega, 1e-9);
    }

    @Test
    void afterResetItStartsFromTheMeasuredVelocity() {
        SwerveAccelLimiter limiter = new SwerveAccelLimiter(LIMITS, DT);
        ChassisVelocities fast = new ChassisVelocities(4.0, 0.0, 0.0);
        step(limiter, STOPPED, STOPPED);   // last target: stopped

        // The robot was stopped with stop() (not limited), then pushed or
        // coasting at 4 m/s. After reset() the limiter must start from 4 m/s,
        // not from its old "stopped" target.
        limiter.reset();
        ChassisVelocities v = step(limiter, fast, fast);
        assertFalse(limiter.wasLimited());
        assertEquals(4.0, v.vx, 1e-9);
    }
}
