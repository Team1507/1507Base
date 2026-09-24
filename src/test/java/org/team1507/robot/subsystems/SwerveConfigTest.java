//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.subsystems;

import static org.junit.jupiter.api.Assertions.fail;

import java.util.List;

import org.junit.jupiter.api.Test;

// ─────────────────────────────────────────────────────────────────────────────
// SwerveConfigTest
//
// Build-time validation of the Tuner X paste zone in Swerve.java. Runs as part
// of `./gradlew build`, so a bad paste fails the build before it can be deployed.
//
// The checks themselves live in Swerve.configProblems(), so the robot runs the
// same checks at startup and prints any problem to the Driver Station.
//
// What it catches (all compile fine but break the robot):
//   - drive gains in the wrong units (the 2026 kV = 2.75 bug)
//   - wheel diameter typed where radius belongs
//   - kSpeedAt12Volts not matching the gear ratio and wheel size
//   - duplicate CAN IDs
//   - encoder offsets in degrees instead of rotations
//   - modules in the wrong corner (kinematics order is FL, FR, BL, BR)
//   - missing or backwards supply current limits
//
// TO ADD A CHECK: add it to Swerve.configProblems(), not here.
// ─────────────────────────────────────────────────────────────────────────────
class SwerveConfigTest {

    @Test
    void pastedSwerveConfigIsSane() {
        List<String> problems = Swerve.configProblems();
        if (!problems.isEmpty()) {
            fail("Swerve config problems (see the Tuner X paste zone in Swerve.java):\n  - "
                + String.join("\n  - ", problems));
        }
    }
}
