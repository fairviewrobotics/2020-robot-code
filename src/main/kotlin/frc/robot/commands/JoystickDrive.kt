/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj.Joystick

/**
 * Drive the drivetrain based on a joystick
 */
class JoystickDrive(val driveSubsystem: DrivetrainSubsystem, val joystick: Joystick) : CommandBase() {
    init {
        addRequirements(driveSubsystem)
    }

    override fun execute() {
        driveSubsystem.driveArcade(joystick.getY(), joystick.getX())
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.driveArcade(0.0, 0.0, true)
    }

    override fun isFinished() = false
}
