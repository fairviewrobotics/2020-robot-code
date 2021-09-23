/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj.Timer
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.GateSubsystem
import frc.robot.subsystems.IndexerSubsystem
import frc.robot.subsystems.IntakeSubsystem
import frc.robot.subsystems.ShooterSubsystem

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
        /* wait for a time to run gate to allow  */
        if (timer.get() > Constants.kShooterSpinUpTime) {
            shooterSubsystem.setSpeed(Constants.kShooterSpeed)
            /* pulse gate to slow down balls going into shooter */
            if (timer.get() % Constants.kGatePulseTime < (Constants.kGatePulseTime * Constants.kGatePulseWidth)) {
                gateSubsystem.setSpeed(Constants.kGateSpeed)
            } else {
                gateSubsystem.setSpeed(0.0)
            }
        } else if (timer.get() > Constants.kGateRDelay){
            shooterSubsystem.setSpeed(Constants.kShooterSpeed)
        } else {
            gateSubsystem.setSpeed(Constants.kGateRSpeed)
        }
    }

    override fun end(interrupted: Boolean) {
        gateSubsystem.setSpeed(0.0)
        shooterSubsystem.setSpeed(0.0)
        intakeSubsystem.setSpeed(0.0)
        indexerSubsystem.setSpeed(0.0)
        timer.stop()
        timer.reset()
    }

    override fun isFinished() =
        if(time == 0.0) false else timer.get() >= time
}
