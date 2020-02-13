/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.CommandBase

/**
 * Drive the drivetrain based on a joystick
 */
class FixedIntakeSpeed(val intakeSubsystem: IntakeSubsystem, val speed: () -> Double) : CommandBase() {
    init {
        addRequirements(intakeSubsystem)
    }

    override fun execute() {
        intakeSubsystem.setSpeed(speed())
    }

    override fun end(interrupted: Boolean) {
        intakeSubsystem.setSpeed(0.0)
    }

    override fun isFinished() = false
}
