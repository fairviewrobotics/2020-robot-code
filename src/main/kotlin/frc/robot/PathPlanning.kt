package frc.robot

import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.RamseteCommand
import edu.wpi.first.wpilibj2.command.Subsystem
import frc.robot.subsystems.DrivetrainSubsystem

// Paths in paths/

fun pathPlanningCommand(trajectoryJSONPath: String, drivetrain: DrivetrainSubsystem): Command {
    val autoVoltageConstraint = DifferentialDriveVoltageConstraint(
        SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter
        ),
        Constants.kDriveKinematics,
        10.0)

    val config = TrajectoryConfig(
        Constants.kMaxSpeedMetersPerSecond,
        Constants.kMaxAccelerationMetersPerSecondSquared
    ).setKinematics(Constants.kDriveKinematics).addConstraint(autoVoltageConstraint)

    val trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSONPath)
    val trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath)

    return RamseteCommand(
        trajectory,
        drivetrain::getPose,
        RamseteController(
            Constants.kRamseteB,
            Constants.kRamseteZeta
        ),
        SimpleMotorFeedforward(
            Constants.ksVolts,
            Constants.kvVoltSecondsPerMeter,
            Constants.kaVoltSecondsSquaredPerMeter
        ),
        Constants.kDriveKinematics,
        drivetrain::getWheelSpeeds,
        PIDController(Constants.kPDriveVel, 0.0, 0.0),
        PIDController(Constants.kPDriveVel, 0.0, 0.0),
        drivetrain::tankDriveVolts,
        arrayOf<Subsystem>(drivetrain)
    )
}

