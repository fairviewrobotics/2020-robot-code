package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.subsystems.DrivetrainSubsystem


class DriveDoubleSpecialSupplier(val driveSubsystem: DrivetrainSubsystem, val forwardSpeed: () -> Double, val turnRadius: () -> Double) : CommandBase() {
    init {
        addRequirements(driveSubsystem)
    }

    override fun execute() {
        driveSubsystem.driveArcade(forwardSpeed(), turnRadius())
    }

    override fun end(interrupted: Boolean) {
        driveSubsystem.driveArcadeSpecial(0.0, 0.0, true)
    }

    override fun isFinished() = false

}