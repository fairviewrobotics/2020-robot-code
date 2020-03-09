package frc.robot.subsystems

import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * Indexer Subsystem
 *
 * Just a indexer motor
 */
class IndexerSubsystem(val motor: SpeedController) : SubsystemBase() {
    override fun periodic() {
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        motor.set(speed)
    }
}
