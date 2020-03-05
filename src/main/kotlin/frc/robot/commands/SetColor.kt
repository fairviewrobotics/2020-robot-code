package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.LEDSubsystem

/**
 *Set the lights to certain colors
 */
class SetColor(val ledSubsystem: LEDSubsystem, val r: Int, val g: Int, val b: Int) : CommandBase() {
    init {
        addRequirements(ledSubsystem)
    }

    override fun execute() {
        for (i in 0 until ledSubsystem.returnLength()) {
            ledSubsystem.setRGB(i, r, g, b)
        }

        ledSubsystem.display()
    }

    override fun end(interrupted: Boolean) {
        ledSubsystem.setRGB(0, 0, 0, 0)
        ledSubsystem.display()
    }

    override fun isFinished() = false
}
