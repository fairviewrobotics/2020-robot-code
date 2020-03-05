package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.LEDSubsystem

/**
 *Set the lights to a rainbow pattern
 */
class SetRainbow(val ledSubsystem: LEDSubsystem) : CommandBase() {
    init {
        addRequirements(ledSubsystem)
    }

    override fun execute() {
        var m_rainbowFirstPixelHue = 0
        for (i in 0 until ledSubsystem.returnLength()) {
            // Calculate the hue 
            var hue = (m_rainbowFirstPixelHue + (i * 180 / ledSubsystem.returnLength())) % 180
            // Set the value
            ledSubsystem.setHSV(i, hue, 255, 128)
        }
        // Increase to make the rainbow "move"
        m_rainbowFirstPixelHue += 3
        // Check bounds
        m_rainbowFirstPixelHue %= 180

        ledSubsystem.display()
    }

    override fun end(interrupted: Boolean) {
        ledSubsystem.setRGB(0, 0, 0, 0)
        ledSubsystem.display()
    }

    override fun isFinished() = false
}
