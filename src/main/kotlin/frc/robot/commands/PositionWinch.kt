/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants
import frc.robot.subsystems.WinchSubsystem

/**
 * Drive the drivetrain based on a joystick
 */
class PositionWinch(val winchSubsystem: WinchSubsystem,
                    val targetPosition: Double) : PIDCommand(
    PIDController(
        Constants.constants["WinchPID_P"] ?: 0.035,
        Constants.constants["WinchPID_I"] ?: 0.0,
        Constants.constants["WinchPID_D"] ?: 0.005),
    winchSubsystem::getPosition,
    targetPosition,
    { output: Double -> winchSubsystem.setSpeed(output) },
    arrayOf(winchSubsystem)) {
    init {
        addRequirements(winchSubsystem)
    }

    /*
    override fun execute() {
        winchSubsystem.setSpeed(speed())
    }*/

    /*
    override fun end(interrupted: Boolean) {
        winchSubsystem.setSpeed(0.0)
    }*/

    override fun isFinished(): Boolean {
        // check if the position is tolerable
        val error = winchSubsystem.getPosition() - targetPosition
        return error <= Constants.constants["WinchPID_PositionTolerance"] ?: 0.0
    }
}
