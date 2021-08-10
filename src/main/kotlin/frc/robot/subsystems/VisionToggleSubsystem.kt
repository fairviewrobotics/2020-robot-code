package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase

enum class VisionModes { BALL, HIGHGOAL }

class VisionToggleSubsystem() : SubsystemBase() {
    var visionMode = VisionModes.BALL
    var visionIntakeOn = true

    override fun periodic() {
    }
}