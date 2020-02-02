/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// An example command. Ideally copy and paste every time you want to create a new command.

package frc.robot.commands

// Import all the required modules: The commands, subsystems, other libraries required in the init
import edu.wpi.first.wpilibj.controller.PIDController
import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants

class ShooterCommand(val shooter: ShooterSubsystem,val speed: Double) : PIDCommand(
  PIDController(Constants.ShooterP, Constants.ShooterI, Constants.ShooterD),
  shooter::getVelocity,
  speed,
  {output: Double -> shooter.setSpeed(output)},
  arrayOf(shooter)) {
  // In the init, between the parenthesis, include all the subsystems and modules
  // you must include. Optionally, replace CommandBase with PIDCommand, for
  // a PID command
  /**
   * Creates a new ExampleCommand.
   *
   * @param m_subsystem The subsystem used by this command.
   */
  init {
    getController().enableContinuousInput(-180.0, 180.0)
    getController().setTolerance(Constants.ShooterToleranceDeg, Constants.ShooterToleranceDegPerS)
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    return getController().atSetpoint()
  }
}
