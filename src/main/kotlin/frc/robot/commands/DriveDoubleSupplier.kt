/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem

/**
 * Drive the drivetrain based on a joystick
 */
class DriveDoubleSupplier(val driveSubsystem: DrivetrainSubsystem, val forwardSpeed: () -> Double, val turnSpeed: () -> Double) : CommandBase() {
    init {
        addRequirements(driveSubsystem)
    }

    override fun execute() {
        driveSubsystem.driveArcade(forwardSpeed(), turnSpeed())
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.driveArcade(0.0, 0.0, true)
        // stops drivetrain by sending 0 speeds
    }

    override fun isFinished() = false
    // should never be finished 
}
