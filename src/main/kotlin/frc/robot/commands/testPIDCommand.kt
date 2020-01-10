/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import frc.robot.subsystems.*
import frc.robot.Constants
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj2.command.PIDCommand
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.drive.DifferentialDrive

class testPIDCommand(val TargetAngle: Double, val drivetrain = DrivetrainSubsystem(DifferentialDrive)) : PIDCommand(
    PIDController(Constants.Drive_Kp,Constants.Drive_Ki,Constants.Drive_Kd),
    drivetrain::getHeading,
    targetAngle,
    output -> drivetrain.arcadeDrive(0,0),
    drive)
    {
  /**
   * Creates a new ExampleCommand.
   *
   * @param m_subsystem The subsystem used by this command.
   */
  init {
    getController().enableContinuousInput(-180,180)
    getController().setTolerance(Constants.Drive_kTurnToleranceDeg)
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    return getController().atSetpoint();
  }
}
