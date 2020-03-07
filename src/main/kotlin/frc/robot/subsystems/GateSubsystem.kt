package frc.robot.subsystems

import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * Indexer Subsystem
 *
 * Just a indexer motor
 */
class GateSubsystem(val motor: SpeedController) : SubsystemBase() {
    /* default command (don't spin) */
    override fun periodic() {
        motor.set(0.0)
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        motor.set(speed)
    }
}
