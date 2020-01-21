package frc.robot.commands

import edu.wpi.first.wpilibj.controller.PIDController
import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants

class TurnToAngle(val driveSubsystem: DrivetrainSubsystem, targetAngle: Double, forwardSpeed: Double): PIDCommand(
        PIDController(Constants.DrivetrainPID_P, Constants.DrivetrainPID_I, Constants.DrivetrainPID_D),
        driveSubsystem::getAngle,
        targetAngle,
        {output: Double -> driveSubsystem.driveArcade(forwardSpeed, output)},
        arrayOf(driveSubsystem)) {


    init {
        getController().enableContinuousInput(-180.0, 180.0)
        getController().setTolerance(Constants.DrivetrainPID_AngleToleranceDeg, Constants.DrivetrainPID_AngleRateToleranceDegPerS)
    }


    override fun isFinished(): Boolean {
        println("checking isFinished")
        /* if no gyro, fail */
        if(!driveSubsystem.gyroUp()) return true
        /* check if we hit setpoint yet */
        return getController().atSetpoint()
    }

}