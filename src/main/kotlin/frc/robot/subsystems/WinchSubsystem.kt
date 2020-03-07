/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems

import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class WinchSubsystem(val winchMotor: SpeedController) : SubsystemBase() {
    override fun periodic() {
        winchMotor.set(0.0)
    }

    fun setWinch(speed: Double) {
        winchMotor.set(speed)
    }

}
