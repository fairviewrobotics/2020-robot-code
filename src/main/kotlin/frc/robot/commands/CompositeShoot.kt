/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import frc.robot.*
import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.CommandBase
import edu.wpi.first.wpilibj.Timer

/**
 * Run a composite shoot on the whole intake through shooter 
 */
class CompositeShoot(val intakeSubsystem: IntakeSubsystem, val indexerSubsystem: IndexerSubsystem, val gateSubsystem: GateSubsystem, val shooterSubsystem: ShooterSubsystem, val time: Double) : CommandBase() {
    val timer = Timer() 
    init {
        addRequirements(intakeSubsystem)
        addRequirements(indexerSubsystem)
        addRequirements(gateSubsystem)
        addRequirements(shooterSubsystem)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() {
        /* stop intake */
        intakeSubsystem.setSpeed(0.0)
        /* continually run indexer */
        indexerSubsystem.setSpeed(Constants.kIndexerSpeed)
        /* run shooter full speed */
        shooterSubsystem.setSpeed(Constants.kShooterSpeed)
        /* wait for a time to run gate to allow  */
        if(timer.get() > Constants.kShooterSpinUpTime) {
          /* pulse gate to slow down balls going into shooter */
          if(timer.get() % Constants.kGatePulseTime < Constants.kGatePulseTime/2) {
            gateSubsystem.setSpeed(Constants.kGateSpeed)
          } else {
            gateSubsystem.setSpeed(0.0)
          }
        } else {
          gateSubsystem.setSpeed(0.0)
        }
    }

    override fun end(interrupted: Boolean) {
        gateSubsystem.setSpeed(0.0)
        shooterSubsystem.setSpeed(0.0)
        timer.stop()
        timer.reset()
    }

    override fun isFinished() = false
}
