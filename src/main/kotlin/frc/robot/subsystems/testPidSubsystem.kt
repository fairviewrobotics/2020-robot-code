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
import com.kauailabs.navx.frc.AHRS

class testPidSubsystem(val drivetrain: DifferentialDrive, val angle: Double, val smff: SimpleMotorFeedforward, val gyro: AHRS) : PIDSubsystem(PIDController(Constants.subsysP, Constants.subsysI, Constants.subsysD)) {
  /**
   * Creates a new ExampleSubsystem.
   */
    init{
        setSetpoint(angle)
        getController().enableContinuousInput(-180.0,180.0)
        getController().setTolerance(Constants.TurnToAngleleranceDeg,Constants.TurnToAngleRateToleranceDegPerS)
    }
  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */

  override fun getMeasurement(): Double {
      return gyro.getAngle();
  }
  
  override fun useOutput(output:Double, setpoint:Double) {
    drivetrain.arcadeDrive(1.0, output+smff.calculate(setpoint)) // this doesn't do what it should
  }

  fun atSetpoint(): Boolean{
      return getController().atSetpoint() // controller is what?   
  }

  fun gyroUp(): Boolean{
    return gyro.isConnected()
  }
  fun stopDrivetrain() {
      drivetrain.arcadeDrive(0.0,0.0)
  }

  fun driveArcade(forwardSpeed: Double, turnSpeed: Double, squareInputs: Boolean = false) {
    drivetrain.arcadeDrive(forwardSpeed, turnSpeed, squareInputs)
  }


}
