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

/**
 * Drivetrain subsystem
 * TODO: add gyro and encoder handling
 */
class DrivetrainSubsystem(val m_leftMotors: SpeedControllerGroup, val m_rightMotors: SpeedControllerGroup, val gyroscope: AHRS, val m_leftEncoder: Encoder, val m_rightEncoder: Encoder) : SubsystemBase() {
    /* default command (don't drive) */
    val m_drive: DifferentialDrive = DifferentialDrive(m_leftMotors,m_rightMotors)
    var m_odometry = DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()))
    init{
        m_leftEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse)
        m_rightEncoder.setDistancePerPulse(Constants.kEncoderDistancePerPulse)
        resetEncoders()

    }
    override fun periodic() {
        m_drive.arcadeDrive(0.0, 0.0)
        m_odometry.update(Rotation2d.fromDegrees(getHeading()),m_leftEncoder.getDistance(),m_rightEncoder.getDistance())
    }

    fun getPose(): Pose2d {
        return m_odometry.getPoseMeters()
    }

    fun getWheelSpeeds(): DifferentialDriveWheelSpeeds{
        return DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(),m_rightEncoder.getRate())
    }
    fun resetOdometry(pose: Pose2d) {
        resetEncoders()
        m_odometry.resetPosition(pose,Rotation2d.fromDegrees(getHeading()))
    }

    fun tankDriveVolts(leftVolts: Double, rightVolts: Double){
        m_leftMotors.setVoltage(leftVolts)
        m_rightMotors.setVoltage(-rightVolts)
    }

    fun resetEncoders(){
        m_leftEncoder.reset()
        m_rightEncoder.reset()
    }

    fun getAverageEncoderDistance(): Double{
        return (m_leftEncoder.getDistance()+m_rightEncoder.getDistance()) / 2.0
    }

    fun getLeftEncoder(): Encoder{
        return m_leftEncoder
    }

    fun getRightEncoder(): Encoder{
        return m_rightEncoder
    }

    fun setMaxOutput(maxOutput: Double){
        m_drive.setMaxOutput(maxOutput)
    }

    fun zeroHeading(){
        gyroscope.reset()
    }

    fun getHeading(): Double{
        return Math.IEEEremainder(gyroscope.getAngle(),360.0)*(Constants.kGyroReversed)
    }

    fun getTurnRate(): Double{
        return gyroscope.getRate() * (Constants.kGyroReversed)
    }
    /* get angle returned by drivetrain */
    fun getAngle(): Double {
        return gyroscope.getAngle()
    }

    /* return true if gyroscope is connected (gyro commands exit if it is not */
    fun gyroUp(): Boolean {
        return gyroscope.isConnected()
    }

    /* simple drive */
    fun driveArcade(forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean = false) {
        m_drive.arcadeDrive(forwardSpeed, turnSpeed, squareInputs)
    }
}
