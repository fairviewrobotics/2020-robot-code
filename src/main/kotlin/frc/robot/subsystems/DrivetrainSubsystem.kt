package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.Encoder

/**
 * Drivetrain subsystem
 * TODO: add gyro and encoder handling
 */
class DrivetrainSubsystem(val drivetrain: DifferentialDrive, val gyroscope: AHRS) : SubsystemBase() {
    /* default command (don't drive) */
    override fun periodic() {
        drivetrain.arcadeDrive(0.0, 0.0)
    }

    /* get angle returned by drivetrain */
    fun getAngle(): Double {
        return gyroscope.getAngle()
    }

    //fun getLSpeed(): Double{
    //    return encoderL.getRate()
    //}

    //fun getRSpeed():Double{
    //    return encoderR.getRate()
    //}

    //fun getOverallSpeed():Double{
    //    return (encoderR.getRate()+encoderL.getRate())/2
    //}

    /* return true if gyroscope is connected (gyro commands exit if it is not */
    fun gyroUp(): Boolean {
        return gyroscope.isConnected()
    }

    /* simple drive */
    fun driveArcade(forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean = false) {
        drivetrain.arcadeDrive(forwardSpeed, turnSpeed, squareInputs)
    }
}
