/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem
import kotlin.math.abs

/**
 * Drive the drivetrain based on a joystick
 */
class XboxDrive(val driveSubsystem: DrivetrainSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(driveSubsystem)
    }

    fun joystickToSpeed(joystickPosSigned: Double): Double {
        val gainSwitchThresh = 0.25
        val gainSwitchPos = 0.25
        val joystickPos = abs(joystickPosSigned)
        val joystickSign = (joystickPosSigned / joystickPos)
        return if (joystickPos < gainSwitchThresh) {
            joystickPosSigned * (gainSwitchPos / gainSwitchThresh)
        } else {
            ((joystickPos - gainSwitchThresh) * (joystickPos - gainSwitchThresh) * (joystickPos - gainSwitchThresh) * (joystickPos - gainSwitchThresh) * joystickSign * 1.1) + (gainSwitchPos * joystickSign)
        }
    }

    override fun execute() {
        driveSubsystem.driveArcade(-controller.getY(GenericHID.Hand.kLeft), joystickToSpeed(controller.getX(GenericHID.Hand.kRight)), false)
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.driveArcade(0.0, 0.0, true)
    }

    override fun isFinished() = false
}
