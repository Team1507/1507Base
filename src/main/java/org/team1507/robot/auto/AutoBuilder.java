//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╔╝╚██╝  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import org.team1507.robot.subsystems.Swerve;

// ─────────────────────────────────────────────────────────────────────────────
// AutoBuilder
//
// Static registry of subsystem references, so AutoSequence steps can reach the
// subsystems without every auto routine passing them around. Initialized ONCE
// from Robot.java after all subsystems are created.
//
// It holds references only. The commands themselves live on the subsystems
// (e.g. AutoBuilder.swerve.driveToPoint(...)).
//
// HOW TO ADD A NEW SUBSYSTEM EACH YEAR (only if auto routines use it):
//   1. Import your subsystem at the top of this file.
//   2. Add a public static field for it below the existing fields.
//   3. Add it as a parameter to init() and assign it.
//   4. Pass it in the AutoBuilder.init(...) call in Robot.java.
//
// Students should NEVER instantiate this class; use the static fields.
// ─────────────────────────────────────────────────────────────────────────────
public final class AutoBuilder {

    // -------------------------------------------------------------------------
    // Subsystem Registry
    // -------------------------------------------------------------------------

    public static Swerve swerve;

    // ADD NEW SUBSYSTEMS HERE each year (e.g. climber, indexer, turret).

    // -------------------------------------------------------------------------
    // Initialization
    //
    // Called ONCE from Robot.java constructor, after all subsystems are built.
    // Add new subsystem parameters here as the robot grows each year.
    // -------------------------------------------------------------------------

    public static void init(Swerve swerve) {
        AutoBuilder.swerve = swerve;
    }

    // Prevent instantiation — this is a static utility class.
    private AutoBuilder() {}
}