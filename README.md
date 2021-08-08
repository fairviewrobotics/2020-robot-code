# 2020 Robot Code

Code for FRC 2036 Black Knights' 2020 Infinite Recharge robot.

## Java Requirements
This project is compatible with Java 11, which is
not the most recent version of Java. It is 
recommended to use [jEnv](https://github.com/jenv/jenv)
to switch between Java versions.

## Building

This project uses gradle as its build system. The GradleRIO plugin provides a wide number of FRC specific gradle commands (see [here](https://github.com/wpilibsuite/GradleRIO) for details).

The major commands are:
-	`./gradlew build` - build the project
-	`./gradlew deploy` - deploy code to the robot (or `./gradlew build deploy` to do both)
-	`./gradlew riolog` - display the rio log output

Deploying and displaying the log require your computer to be connected to the robot (tethered over ethernet or connected to the robot's radio).

Passing the `--offline` flag to gradle will prevent it from trying to update and/or download dependencies, which can be useful at competition.

It is **highly recommended** to use intellij when working on the code. VSCode, though it is the officially supported editor, provides *very* limited code completion. Intellij provides excellent code completion, especially for kotlin.

## FRC Documentation
The documentation for the FRC Control System can be found [here](https://docs.wpilib.org/en/latest/). These are very useful to refer to.