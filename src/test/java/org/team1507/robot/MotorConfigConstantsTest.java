//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot;

import static org.junit.jupiter.api.Assertions.fail;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.ArrayList;
import java.util.List;

import org.junit.jupiter.api.Test;

import org.team1507.lib.core.util.MotorConfig;

// ─────────────────────────────────────────────────────────────────────────────
// MotorConfigConstantsTest
//
// Build-time check of every motor configuration in Constants.java, like
// NodeBoundsTest does for field positions. Runs as part of `./gradlew build`,
// so a bad motor config fails the build before it can be deployed.
//
// It finds every `static MotorConfig` field in Constants and all of its nested
// classes (kIntake, kShooter, ...) automatically. Adding a new config needs no
// change here.
//
// What it catches (see MotorConfig.problems() for the full list):
//   - a missing or zero supply current limit (brownout protection)
//   - a sustained supply limit set higher than the supply limit
//   - an elevator with no drum diameter (so meters/inches can't work)
//   - PID gains on an open-loop roller (they would be silently ignored)
//   - torque control without FOC (needs Phoenix Pro)
//
// TO ADD A CHECK: add it to MotorConfig.problems(), not here.
// ─────────────────────────────────────────────────────────────────────────────
class MotorConfigConstantsTest {

    @Test
    void everyMotorConfigInConstantsIsSane() throws IllegalAccessException {
        List<String> problems = new ArrayList<>();
        checkClass(Constants.class, problems);

        if (!problems.isEmpty()) {
            fail("Motor config problems in Constants.java:\n  - " + String.join("\n  - ", problems));
        }
    }

    /** Checks every static MotorConfig field in this class, then its nested classes. */
    private static void checkClass(Class<?> cls, List<String> problems) throws IllegalAccessException {
        for (Field field : cls.getDeclaredFields()) {
            if (Modifier.isStatic(field.getModifiers()) && field.getType() == MotorConfig.class) {
                field.setAccessible(true);
                MotorConfig config = (MotorConfig) field.get(null);
                String name = cls.getSimpleName() + "." + field.getName();
                if (config == null) {
                    problems.add(name + " is null");
                    continue;
                }
                for (String problem : config.problems()) {
                    problems.add(name + ": " + problem);
                }
            }
        }
        for (Class<?> nested : cls.getDeclaredClasses()) {
            checkClass(nested, problems);
        }
    }
}
