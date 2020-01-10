/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj2.command.PIDSubsystem
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.Encoder
import frc.robot.Constants

class testPidSubsystem(val drivetrain: DifferentialDrive, val encoder: Encoder, val smff: SimpleMotorFeedforward) : PIDSubsystem(PIDController(Constants.Kp, Constants.Ki, Constants.Kd)) {
  /**
   * Creates a new ExampleSubsystem.
   */

    init{
        encoder.setDistancePerPulse(Constants.dpp)
        setSetpoint(Constants.kTargetRPS)
        controller.setTolerance(Constants.kToleranceRPS)
    }

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  override fun getMeasurement(): Double {
      return encoder.getRate();
  }
  
  override fun useOutput(output:Double, setpoint:Double) {
    drivetrain.arcadeDrive(1.0, output+smff.calculate(setpoint)) // this doesn't do what it should
  }

  fun atSetpoint(): Boolean{
      return controller.atSetpoint() // controller is what?   
  }

  fun stopDrivetrain() {
      drivetrain.arcadeDrive(0.0,0.0)
  }

}
