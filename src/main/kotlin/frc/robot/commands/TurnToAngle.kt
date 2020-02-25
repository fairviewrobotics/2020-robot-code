package frc.robot.commands

import edu.wpi.first.wpilibj.controller.PIDController
import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants

class TurnToAngle(val driveSubsystem: DrivetrainSubsystem, targetAngle: Double, forwardSpeed: Double) : PIDCommand(
    PIDController(
        Constants.constants["DrivetrainPID_P"] ?: 0.035,
        Constants.constants["DrivetrainPID_I"] ?: 0.0,
        Constants.constants["DrivetrainPID_D"] ?: 0.005
    ),
    driveSubsystem::getAngle,
    targetAngle,
    { output: Double -> driveSubsystem.driveArcade(forwardSpeed, output) },
    arrayOf(driveSubsystem)) {


    init {
        controller.enableContinuousInput(-180.0, 180.0)
        /** reload pid parameters from network tables */
        setPIDParams()
    }

    fun setPIDParams() {
        controller.setTolerance(
            Constants.constants["DrivetrainPID_AngleToleranceDeg"] ?: 2.0,
            Constants.constants["DrivetrainPID_AngleRateToleranceDegPerS"] ?: 1.0
        )
        controller.setPID(
            Constants.constants["DrivetrainPID_P"] ?: 0.035,
            Constants.constants["DrivetrainPID_I"] ?: 0.0,
            Constants.constants["DrivetrainPID_D"] ?: 0.005
        )
    }


    override fun isFinished(): Boolean {
        println("checking isFinished")
        /* if no gyro, fail */
        if (!driveSubsystem.gyroUp()) return true
        /* check if we hit setpoint yet */
        return controller.atSetpoint()
    }

}