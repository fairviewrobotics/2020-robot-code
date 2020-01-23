package frc.robot.commands

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.livewindow.LiveWindow
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants

/**
 * The command for the high goal vision targeting algorithm
 * Right now it just drives straight at the target
 */

class VisionHighGoal(val driveSubsystem: DrivetrainSubsystem, forwardSpeed: Double): PIDCommand(
        PIDController(
                Constants.constants["DrivetrainPID_P"] ?: 0.035,
                Constants.constants["DrivetrainPID_I"] ?: 0.0,
                Constants.constants["DrivetrainPID_D"] ?: 0.005
        ),
        driveSubsystem::getAngle,
        {
            /* dampen turning */
            /* because the offsets sum over multiple iterations, dampening doesn't make us not hit the target, it just slows turning towards it */
            var yawOff = yaw.getDouble(0.0)/1.5
            /* dampen even more if we are making more than a small turn. */
            if(yawOff >= 5.0){
                yawOff /= 1.5
            }

            driveSubsystem.getAngle() + yawOff
        },
        {output: Double -> driveSubsystem.driveArcade(forwardSpeed, output)},
        arrayOf(driveSubsystem)) {

    companion object {
        val ntInst = NetworkTableInstance.getDefault()
        val table = ntInst.getTable("high-vision")
        val yaw = table.getEntry("yaw")
        val isTarget = table.getEntry("isTarget")
    }

    init {
        getController().enableContinuousInput(-180.0, 180.0)
        /** reload pid parameters from network tables */
        setPIDParams()
    }

    fun setPIDParams() {
        getController().setTolerance(
                Constants.constants["DrivetrainPID_AngleToleranceDeg"] ?: 2.0,
                Constants.constants["DrivetrainPID_AngleRateToleranceDegPerS"] ?: 1.0
        )
        getController().setPID(
                Constants.constants["DrivetrainPID_P"] ?: 0.035,
                Constants.constants["DrivetrainPID_I"] ?: 0.0,
                Constants.constants["DrivetrainPID_D"] ?: 0.005
        )
    }

    override fun isFinished(): Boolean {

        /* if no gyro, fail */
        if(!driveSubsystem.gyroUp()) return true
        //return getController().atSetpoint()
        /* if we can't see a vision target, stop */
        /* TODO: require no vision target for two or three frames before stopping */
        if(!isTarget.getBoolean(false)) return true
        return false
    }

}