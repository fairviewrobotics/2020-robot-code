/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ClimbSubsystem

class FixedClimbSpeed(val climber: ClimbSubsystem, val speed: () -> Double) : CommandBase() {
    /**
     * @param m_subsystem The subsystem used by this command.
     */
    init {
        addRequirements(climber)
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        climber.setClimber(speed())
    }

    override fun end(interrupted: Boolean) {
        climber.setClimber(0.0)
    }

    // Returns true when the command should end.
    override fun isFinished() = false
}
