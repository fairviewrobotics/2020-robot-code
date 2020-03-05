package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.Encoder
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj.SpeedControllerGroup
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds
import frc.robot.Constants
import kotlin.math.IEEErem

/**
 * Drivetrain subsystem
 */
class DrivetrainSubsystem(val leftSpeedController: SpeedControllerGroup, val rightSpeedController: SpeedControllerGroup, val gyroscope: AHRS, val leftEncoder: Encoder, val rightEncoder: Encoder) : SubsystemBase() {

    val drivetrain = DifferentialDrive(leftSpeedController, rightSpeedController)
    var odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()))

    init{
        leftEncoder.setDistancePerPulse(Constants.kDrivetrainEncoderDistancePerPulse)
        rightEncoder.setDistancePerPulse(Constants.kDrivetrainEncoderDistancePerPulse)
        resetEncoders()
    }

    fun updateOdometry() {
        odometry.update(Rotation2d.fromDegrees(getHeading()), leftEncoder.getDistance(), rightEncoder.getDistance())
    }

    fun getPose() =
        odometry.poseMeters

    fun getWheelSpeeds() =
        DifferentialDriveWheelSpeeds(leftEncoder.rate, rightEncoder.rate)

    fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()))
    }

    fun tankDriveVolts(leftVolts: Double, rightVolts: Double){
        leftSpeedController.setVoltage(leftVolts)
        rightSpeedController.setVoltage(-rightVolts)
    }

    fun resetEncoders(){
        leftEncoder.reset()
        rightEncoder.reset()
    }

    fun getAverageEncoderDistance() =
        (leftEncoder.distance + rightEncoder.distance) / 2.0

    fun setMaxOutput(maxOutput: Double){
        drivetrain.setMaxOutput(maxOutput)
    }

    override fun periodic() {
        drivetrain.arcadeDrive(0.0, 0.0, false)
        updateOdometry()
    }

    fun resetGyro(){
        gyroscope.reset()
    }

    /* get angle returned by gyroscope */
    fun getAngle(): Double {
        return gyroscope.angle
    }

    /* get normalized angle returned by gyroscope */
    fun getHeading(): Double{
        return gyroscope.angle.IEEErem(360.0) * Constants.kGyroReversed
    }

    fun getTurnRate(): Double {
        return gyroscope.rate * Constants.kGyroReversed
    }

    /* return true if gyroscope is connected (gyro commands exit if it is not */
    fun gyroUp(): Boolean {
        return gyroscope.isConnected
    }

    /* simple drive */
    fun driveArcade(forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean = false) {
        setMaxOutput(1.0)
        updateOdometry()
        drivetrain.arcadeDrive(forwardSpeed, turnSpeed, squareInputs)
    }
}
