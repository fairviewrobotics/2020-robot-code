/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.GateSubsystem

/**
 * Run gate subsystem at a fixed speed
 */
class FixedGateSpeed(val gateSubsystem: GateSubsystem, val speed: () -> Double) : CommandBase() {
    init {
        addRequirements(gateSubsystem)
    }

    override fun execute() {
        gateSubsystem.setSpeed(speed())
    }

    override fun end(interrupted: Boolean) {
        gateSubsystem.setSpeed(0.0)
    }

    override fun isFinished() = false
}
