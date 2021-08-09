package frc.robot.commands

import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants
import frc.robot.subsystems.DrivetrainSubsystem

/**
 * The command for the ball vision targeting algorithm
 * Right now it just drives straight at the balls.
 * This code is almost entirely copied from VisionHighGoal.kt
 */

class BallVision(val driveSubsystem: DrivetrainSubsystem, forwardSpeed: Double) : PIDCommand(
        PIDController(
                Constants.constants["DrivetrainPID_P"] ?: 0.035,
                Constants.constants["DrivetrainPID_I"] ?: 0.0,
                Constants.constants["DrivetrainPID_D"] ?: 0.005
        ),
        driveSubsystem::getAngle,
        {
            /* dampen turning */
            /* because the offsets sum over multiple iterations, dampening doesn't make us not hit the target, it just slows turning towards it */
            var yawOff = yaw.getDouble(0.0) / 1.3
            /* dampen even more if we are making more than a small turn. */
            if (yawOff >= 5.0) {
                yawOff /= 1.5
            }

            driveSubsystem.getAngle() + yawOff
        },
        { output: Double -> driveSubsystem.driveArcade(forwardSpeed, output) },
        arrayOf(driveSubsystem)) {

    var pTarget = true
    var ppTarget = true

    companion object {
        val ntInst = NetworkTableInstance.getDefault()
        val table = ntInst.getTable("ball-vision")
        val yaw = table.getEntry("yaw") // ideal yaw to collect most balls
        val ballFound = table.getEntry("ballFound") // whether a ball is visible
        val ballHeight = table.getEntry("ballHeight") // the height of the ball
    }

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
        /* if no gyro, fail */
        if (!driveSubsystem.gyroUp()) return true


        /* if we can't see a vision target, stop */
        val targetFound = ballFound.getBoolean(false)
        if (!targetFound && !pTarget && !ppTarget) return true

        val tmp = pTarget
        pTarget = targetFound
        ppTarget = tmp

        return false
    }

}