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
import kotlin.math.pow

/**
 * Drive the drivetrain based on a joystick
 */
class XboxDriveSpecial(val driveSubsystem: DrivetrainSubsystem, val controller: XboxController) : CommandBase() {
    init {
        addRequirements(driveSubsystem)
    }

    fun joystickToSpeed(joystickPosSigned: Double): Double {
        val gainSwitchThresh = 0.25
        val gainSwitchPos = 0.25

        val modPower = 4
        val joystickPos = abs(joystickPosSigned)
        val joystickSign = (joystickPosSigned / joystickPos)
        return if (joystickPos < gainSwitchThresh) {
            joystickPosSigned * (gainSwitchPos / gainSwitchThresh)
        } else {
            ((joystickPos - gainSwitchThresh).pow(modPower) * joystickSign * 1.1) + (gainSwitchPos * joystickSign)
        }
    }

    fun joystickToRadius(joystickPosSigned: Double) : Double {
        val gainSwitchThresh = 0.25
        val gainSwitchPos = 0.25

        val radiusMod = 2 // make this a constant after tuning is finished!
        val radiusPower = 2 // see above

        val joystickPos = abs(joystickPosSigned)
        val joystickPosSign = (joystickPosSigned / joystickPos)
        return if (joystickPos < gainSwitchThresh) {
            joystickPosSigned * (gainSwitchPos / gainSwitchThresh) // probably bad but eh
        } else {
            radiusMod * joystickPosSign * (1/(joystickPos - gainSwitchThresh).pow(radiusPower) - 1 + (gainSwitchPos * joystickPosSign))
        }
    }


    override fun execute() {
        driveSubsystem.driveArcadeSpecial(-controller.getY(GenericHID.Hand.kLeft), joystickToRadius(controller.getX(GenericHID.Hand.kRight)), false)
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.driveArcadeSpecial(0.0, 0.0, true)
    }

    override fun isFinished() = false
}
