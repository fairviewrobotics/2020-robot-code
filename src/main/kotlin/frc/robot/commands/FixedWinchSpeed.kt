/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.WinchSubsystem

class FixedWinchSpeed(val winch: WinchSubsystem, val speed: () -> Double) : CommandBase() {
    /**
     * @param m_subsystem The subsystem used by this command.
     */
    init {
        addRequirements(winch)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        winch.setWinch(speed())
    }

    // Returns true when the command should end.
    override fun isFinished() = false
}
