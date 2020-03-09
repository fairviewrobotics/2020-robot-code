package frc.robot.subsystems

import com.revrobotics.CANEncoder
import com.revrobotics.CANPIDController
import com.revrobotics.CANSparkMax
import edu.wpi.first.wpilibj2.command.SubsystemBase

/**
 * Shooter Subsystem
 *
 * Just a flywheel motor
 */
class ShooterSubsystem(val flywheelMotor: CANSparkMax) : SubsystemBase() {
    override fun periodic() {
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        flywheelMotor.set(speed)
    }

    fun getPidController(): CANPIDController {
        return flywheelMotor.getPIDController()
    }

    fun getEncoder(): CANEncoder {
        return flywheelMotor.getEncoder()
    }

    fun pidWrite(output: Double) {
        flywheelMotor.pidWrite(output)
    }
}