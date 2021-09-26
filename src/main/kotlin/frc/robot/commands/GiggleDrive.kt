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
import frc.robot.subsystems.DrivetrainSubsystem

/**
 * Run drivetrains back and forth slightly
 */
class GiggleDriveCommand(val drive: DrivetrainSubsystem) : CommandBase() {
    val timer = Timer()

    init {
        addRequirements(drive)
    }

    override fun initialize() {
        timer.reset()
        timer.start()
    }

    override fun execute() {
        if(timer.get() % 0.35 < 0.175) {
            drive.driveArcade(0.25, 0.0, false)
        } else {
            drive.driveArcade(-0.25, 0.0, false)
        }
    }

    override fun end(interrupted: Boolean) {
        timer.stop()
        timer.reset()
    }

    override fun isFinished() = false
}
