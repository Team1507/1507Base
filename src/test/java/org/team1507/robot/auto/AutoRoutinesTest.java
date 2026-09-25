//  ██╗    ██╗ █████╗ ██████╗ ██╗      ██████╗  ██████╗██╗  ██╗███████╗
//  ██║    ██║██╔══██╗██╔══██╗██║     ██╔═══██╗██╔════╝██║ ██╔╝██╔════╝
//  ██║ █╗ ██║███████║██████╔╝██║     ██║   ██║██║     █████╔╝ ███████╗
//  ██║███╗██║██╔══██║██╔══██╗██║     ██║   ██║██║     ██╔═██╗ ╚════██║
//  ╚███╔███╔╝██║  ██║██║  ██║███████╗╚██████╔╝╚██████╗██║  ██╗███████║
//   ╚══╝╚══╝ ╚═╝  ╚═╝╚═╝  ╚═╝╚══════╝ ╚═════╝  ╚═════╝╚═╝  ╚═╝╚══════╝
//                           TEAM 1507 WARLOCKS

package org.team1507.robot.auto;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.fail;

import java.io.File;
import java.lang.reflect.Constructor;
import java.lang.reflect.Method;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import org.junit.jupiter.api.Test;

import org.wpilib.command3.Command;
import org.wpilib.opmode.Autonomous;

import org.team1507.lib.core.framework.AutoOpMode;
import org.team1507.robot.Robot;
import org.team1507.robot.TestRobot;

// ─────────────────────────────────────────────────────────────────────────────
// AutoRoutinesTest
//
// Builds EVERY auto routine in robot/auto/routines on every build, the same way
// the robot does when the routine is enabled. The build fails if a routine:
//   - has a mistake AutoSequence refuses (a misplaced modifier, a drivetrain
//     step while the robot is still driving, ...), or
//   - has a route warning (a classic leg through a field element, policy nodes
//     more than 5 m apart).
// So a broken auto never reaches the robot. Nothing to register: new routine
// files are found automatically.
//
// Policy autos are skipped until the policy driver is installed on the robot
// (AutoBuilder.policyDriver); until then they are expected to refuse to build.
// ─────────────────────────────────────────────────────────────────────────────
class AutoRoutinesTest {

    private static final String PACKAGE = "org.team1507.robot.auto.routines";

    @Test
    void everyAutoRoutineBuildsWithoutMistakesOrRouteWarnings() throws Exception {
        Robot robot = TestRobot.get();
        List<Class<?>> routines = findRoutines();
        assertFalse(routines.isEmpty(), "no @Autonomous classes found in " + PACKAGE);

        List<String> problems = new ArrayList<>();
        for (Class<?> routine : routines) {
            String name = routine.getSimpleName();
            try {
                AutoOpMode opMode = construct(routine, robot);
                Method build = AutoOpMode.class.getDeclaredMethod("build");
                build.setAccessible(true);
                Command command = (Command) build.invoke(opMode);
                if (command == null) {
                    problems.add(name + ": build() returned null");
                } else if (!AutoSequence.warningsFromLastBuild().isEmpty()) {
                    problems.add(name + ": " + String.join("; ", AutoSequence.warningsFromLastBuild()));
                }
            } catch (java.lang.reflect.InvocationTargetException e) {
                Throwable cause = e.getCause();
                boolean policyNotInstalled = cause.getMessage() != null
                    && cause.getMessage().contains("policy driver")
                    && AutoBuilder.policyDriver == null;
                if (!policyNotInstalled) {
                    problems.add(name + ": " + cause.getMessage());
                }
            }
        }
        if (!problems.isEmpty()) {
            fail("Auto routines with problems:\n  " + String.join("\n  ", problems));
        }
    }

    /** Every @Autonomous class in the routines package. */
    private static List<Class<?>> findRoutines() throws Exception {
        List<Class<?>> found = new ArrayList<>();
        String path = PACKAGE.replace('.', '/');
        for (URL url : Collections.list(AutoRoutinesTest.class.getClassLoader().getResources(path))) {
            if (!"file".equals(url.getProtocol())) {
                continue;
            }
            File[] files = new File(url.toURI()).listFiles((dir, file) -> file.endsWith(".class") && !file.contains("$"));
            if (files == null) {
                continue;
            }
            for (File file : files) {
                Class<?> cls = Class.forName(PACKAGE + "." + file.getName().replace(".class", ""));
                if (cls.isAnnotationPresent(Autonomous.class) && AutoOpMode.class.isAssignableFrom(cls)) {
                    found.add(cls);
                }
            }
        }
        return found;
    }

    /** Constructs a routine the way WPILib does: a Robot constructor if it has one, else no arguments. */
    private static AutoOpMode construct(Class<?> routine, Robot robot) throws Exception {
        for (Constructor<?> constructor : routine.getConstructors()) {
            Class<?>[] parameters = constructor.getParameterTypes();
            if (parameters.length == 1 && parameters[0].isAssignableFrom(Robot.class)) {
                return (AutoOpMode) constructor.newInstance(robot);
            }
        }
        return (AutoOpMode) routine.getConstructor().newInstance();
    }
}
