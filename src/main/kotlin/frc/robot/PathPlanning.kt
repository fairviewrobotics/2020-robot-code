package frc.robot

import edu.wpi.first.wpilibj.Filesystem
import edu.wpi.first.wpilibj.controller.PIDController
import edu.wpi.first.wpilibj.controller.RamseteController
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward
import edu.wpi.first.wpilibj.geometry.Pose2d
import edu.wpi.first.wpilibj.geometry.Rotation2d
import edu.wpi.first.wpilibj.geometry.Translation2d
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint
import edu.wpi.first.wpilibj2.command.*
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
    val trajectoryUntransformed = TrajectoryUtil.fromPathweaverJson(trajectoryPath)



    val transform = drivetrain.getPose().minus(trajectoryUntransformed.getInitialPose())
    val trajectory = trajectoryUntransformed.transformBy(transform)

    /*val trajectory = TrajectoryGenerator.generateTrajectory(
        // Start at the origin facing the +X direction
        Pose2d(0.0, 0.0, Rotation2d(0.0)),
    // Pass through these two interior waypoints, making an 's' curve path
    listOf(
        Translation2d(0.5, 0.5),
        Translation2d(1.0, -0.5)
    ),
    // End 3 meters straight ahead of where we started, facing forward
    Pose2d(1.5, 0.0, Rotation2d(0.0)),
    // Pass config
    config
    )*/

    return SequentialCommandGroup (
        InstantCommand({drivetrain.resetOdometry(Pose2d(0.0, 0.0, Rotation2d(0.0)))}, arrayOf(drivetrain)),
        RamseteCommand(
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
    )
}

