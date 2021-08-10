/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.ExampleSubsystem
import frc.robot.subsystems.VisionModes
import frc.robot.subsystems.VisionToggleSubsystem

class VisionModeChange(val visionToggle: VisionToggleSubsystem,
                       val mode: () -> Pair<VisionModes, Boolean>) : CommandBase() {
    /**
     * Creates a new VisionModeChange command.
     *
     * @param visionToggle The subsystem used by this command.
     * @param mode The function to get the vision mode and automatic intake state as a pair
     */
    init {
        addRequirements(visionToggle)
    }

    // Called when the command is initially scheduled.
    override fun initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        val currentMode = mode()
        visionToggle.visionMode = currentMode.first
        visionToggle.visionIntakeOn = currentMode.second
    }

    // Called once the command ends or is interrupted.
    override fun end(interrupted: Boolean) {
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
