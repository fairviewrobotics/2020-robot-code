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

class testPIDCommand(val TargetAngle: Double, val drivetrain: DrivetrainSubsystem) : PIDCommand(
    PIDController(Constants.TurnToAngleP,Constants.TurnToAngleI,Constants.TurnToAngleD),
    drivetrain::getAngle,
    TargetAngle,
    {output:Double -> drivetrain.driveArcade(0.0,output)},
    arrayOf(drivetrain)
)
    {
  /**
   * Creates a new ExampleCommand.
   *
   * @param m_subsystem The subsystem used by this command.
   */
  init {
    getController().enableContinuousInput(-180.0,180.0)
    getController().setTolerance(Constants.TurnToAngleleranceDeg,Constants.TurnToAngleRateToleranceDegPerS)
  }

  // Returns true when the command should end.
  override fun isFinished(): Boolean {
    if(!drivetrain.gyroUp()) return true
    return getController().atSetpoint();
  }
}
