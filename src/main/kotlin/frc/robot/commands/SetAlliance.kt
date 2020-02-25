package frc.robot.commands

import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.CommandBase

/**
 *Set the lights to certain colors
 */
class SetAlliance(val ledSubsystem: LEDSubsystem) : CommandBase() {
    init {
        addRequirements(ledSubsystem)
    }

    override fun execute() {
        if (ledSubsystem.returnAlliance() == "Red") {

            for (i in 0 until ledSubsystem.returnLength()) {
                ledSubsystem.setRGB(i, 255, 0, 0)
            }

            ledSubsystem.display()

        } else if (ledSubsystem.returnAlliance() == "Blue") {

            for (i in 0 until ledSubsystem.returnLength()) {
                ledSubsystem.setRGB(i, 0, 0, 255)
            }

            ledSubsystem.display()
        } else {

            for (i in 0 until ledSubsystem.returnLength()) {
                ledSubsystem.setRGB(i, 255, 255, 255)
            }

            ledSubsystem.display()

        }
    }

    override fun end(interrupted: Boolean) {
        ledSubsystem.setRGB(0, 0, 0, 0)
        ledSubsystem.display()
    }

    override fun isFinished() = false
}
