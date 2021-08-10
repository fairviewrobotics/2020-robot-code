/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants
import frc.robot.subsystems.*
import java.time.LocalTime

class ChangeDriveMode(val driveToggle: DriveModeToggleSubsystem,
                       val activating: () -> Boolean) : CommandBase() {
    /**
     * Creates a new ChangeDriveMode command.
     *
     * @param driveToggle The subsystem used by this command.
     * @param activating The function to call to determine if the driver
     *   is trying to change the mode.
     */

    init {
        addRequirements(driveToggle)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val now = LocalTime.now()

        // handle input
        if (activating()) {
            if (driveToggle.holdStartTime == null) {
                driveToggle.holdStartTime = now
            } else if (now.minusSeconds(Constants.kDriveToggleHoldTime.toLong()).isAfter(driveToggle.holdStartTime)) {
                driveToggle.incrementMode()
                driveToggle.holdStartTime = null
            }
        } else driveToggle.holdStartTime = null

        // update the drive mode
        driveToggle.applyDriveMode()

        // stop rumble if time is up
        if (driveToggle.rumbleStartTime != null
                && now.minusSeconds(Constants.kDriveModeChangeRumbleTime.toLong())
                        .isAfter(driveToggle.rumbleStartTime)) {
            driveToggle.stopRumble()
        }
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
