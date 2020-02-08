/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj.XboxController

/**
 * Drive the drivetrain based on a joystick
 */
class XboxDrive(val driveSubsystem: DrivetrainSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(driveSubsystem)
    }

    override fun execute() {
        driveSubsystem.driveArcade(-controller.getY(), controller.getX())
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.driveArcade(0.0, 0.0, true)
    }

    override fun isFinished() = false
}
