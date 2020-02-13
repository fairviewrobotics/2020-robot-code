/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// An example command. Ideally copy and paste every time you want to create a new command.

package frc.robot.commands

// Import all the required modules: The commands, subsystems, other libraries required in the init
import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants

class ShooterCommand(val shooter: ShooterSubsystem,val speed: Double) : CommandBase() {
  // In the init, between the parenthesis, include all the subsystems and modules
  // you must include. Optionally, replace CommandBase with PIDCommand, for
  // a PID command
  init {
    pid = shooter.getPidController()
    pid.setP(Constants.ShooterP)
    pid.setI(Constants.ShooterI)
    pid.setD(Constants.ShooterD)
    pid.setFF(Constants.ShooterFF)
    pid.setReference(speed,ctrl)//TODO: Figure out the correct control mode
  }

  override fun execute(){
    shooter.
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    return getController().atSetpoint()
  }
}
