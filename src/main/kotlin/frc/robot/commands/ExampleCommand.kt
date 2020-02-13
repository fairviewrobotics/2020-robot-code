/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// An example command. Ideally copy and paste every time you want to create a new command.

package frc.robot.commands

// Import all the required modules: The commands, subsystems, other libraries required in the init
import frc.robot.subsystems.ExampleSubsystem
import edu.wpi.first.wpilibj2.command.CommandBase

class ExampleCommand(val m_subsystem: ExampleSubsystem) : CommandBase() {
  // In the init, between the parenthesis, include all the subsystems and modules
  // you must include. Optionally, replace CommandBase with PIDCommand, for
  // a PID command
  /**
   * Creates a new ExampleCommand.
   *
   * @param m_subsystem The subsystem used by this command.
   */
  init {
    addRequirements(m_subsystem)
  }

  // Called when the command is initially scheduled.
  override fun initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  override fun execute() {
  }

  // Called once the command ends or is interrupted.
  override fun end(interrupted: Boolean) {
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    return false
  }
}
