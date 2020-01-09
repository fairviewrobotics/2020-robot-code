package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.drive.DifferentialDrive

/**
 * Drivetrain subsystem
 * TODO: add gyro and enocder handling
 */
class DrivetrainSubsystem(val drivetrain: DifferentialDrive) : SubsystemBase() {
    /* default command (don't drive) */
    override fun periodic() {
        drivetrain.arcadeDrive(0.0, 0.0)
    }

    /* simple drive */
    fun driveArcade(forwardSpeed: Double, turnSpeed: Double) {
        drivetrain.arcadeDrive(forwardSpeed, turnSpeed)
    }
}
