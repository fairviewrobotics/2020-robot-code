package frc.robot.subsystems

import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * Shooter Subsystem
 *
 * Just a flywheel motor
 */
class ShooterSubsystem(val flywheelMotor: CANSparkMax) : SubsystemBase() {
    /* default command (don't spin) */
    override fun periodic() {
        flywheelMotor.set(0.0)
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        flywheelMotor.set(speed)
    }
}
