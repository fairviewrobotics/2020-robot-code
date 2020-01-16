/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj.controller.PIDController
import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.PIDCommand
import frc.robot.Constants

class TurnToAngle(val driveSubsystem: DrivetrainSubsystem, targetAngle: Double): PIDCommand(
        PIDController(Constants.TurnToAngleP, Constants.TurnToAngleI, Constants.TurnToAngleD),
        driveSubsystem::getAngle,
        targetAngle,
        {output: Double -> driveSubsystem.driveArcade(0.0, output)},
        arrayOf(driveSubsystem)) {

    init {
        getController().enableContinuousInput(-180.0, 180.0)
        getController().setTolerance(Constants.TurnToAngleleranceDeg, Constants.TurnToAngleRateToleranceDegPerS)
    }

    override fun isFinished(): Boolean {
        if(!driveSubsystem.gyroUp()) return true
        return getController().atSetpoint()
    }

}