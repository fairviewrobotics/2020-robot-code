/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import frc.robot.commands.*
import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.DriverStation
import edu.wpi.first.wpilibj.SpeedControllerGroup
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.GenericHID.Hand.*
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj.XboxController.Button.*

import com.ctre.phoenix.motorcontrol.can.*
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup

/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
  // The robot's subsystems and commands are defined here...

  var m_autoCommandChooser: SendableChooser<Command> = SendableChooser()

  /* controller0 - secondary driver controller */
  val controller0 = XboxController(0)
  /* controller1 - primary driver controller (overriden by controller0) */
  val controller1 = XboxController(1)

  /** --- setup drivetrain --- **/
  val motorFrontLeft =  WPI_TalonSRX(5)
  val motorBackLeft =   WPI_TalonSRX(7)
  val motorFrontRight = WPI_TalonSRX(8)
  val motorBackRight =  WPI_TalonSRX(9)

  /* keep speeds same on motors on each side */
  val motorsLeft = SpeedControllerGroup(motorFrontLeft, motorBackLeft)
  val motorsRight = SpeedControllerGroup(motorFrontRight, motorBackRight)

  val gyro = AHRS()

  val drivetrain = DrivetrainSubsystem(DifferentialDrive(motorsLeft, motorsRight), gyro)
  val shooter = ShooterSubsystem(CANSparkMax(10, MotorType.kBrushless))
  val intake = IntakeSubsystem(WPI_TalonSRX(4))
  val indexer = IndexerSubsystem(WPI_TalonSRX(2))
  val gate = GateSubsystem(WPI_TalonSRX(3))
  val lights = LEDSubsystem(AddressableLED(0), 60, DriverStation.getInstance())

  /*** --- commands --- ***/
  //drive by a joystick (controller1)
  val XboxDriveCommand = XboxDrive(drivetrain, controller1)

  /** -- 0 point autos -- **/
  val noAuto = DriveDoubleSupplier(drivetrain, { 0.0 }, { 0.0 })

  /** --- 5 point autos --- **/
  //backup simple auto
  val backupAuto = DriveDoubleSupplier(drivetrain, { 0.3 }, { 0.0 }).withTimeout(2.0)
  //forward simple auto
  val forwardAuto = DriveDoubleSupplier(drivetrain, { -0.3 }, { 0.0 }).withTimeout(2.0)

  //backup and SPIN!! (looking cool is basically the same thing as winning)
  val spinAuto = SequentialCommandGroup(
          DriveDoubleSupplier(drivetrain, { 0.3 }, { 0.0 }).withTimeout(2.0),
          DriveDoubleSupplier(drivetrain, { 0.0 }, { 0.5 }).withTimeout(12.0)
  )

  /** -- more than 5 point autos (hopefully) -- **/
  // power port vision
  val visionHighGoalCommand = SequentialCommandGroup(
          VisionHighGoal(drivetrain, -0.3),
          DriveDoubleSupplier(drivetrain, { -0.3 }, { 0.0 }).withTimeout(0.5)
  )

  /* for testing PID loops */
  val turnToAngleCommand = TurnToAngle(drivetrain, 90.0, 0.0)

  /* for running shooter */
  val shooterCommand = FixedShooterSpeed(shooter, {0.5})

  /*LED Lights */
  val setAlliance = SetAlliance(lights)
  val setRed = SetColor(lights, 255, 0, 0)
  val setBlue = SetColor(lights, 0,0,255)
  val setWhite = SetColor(lights, 255,255,255)
  val setRainbow = SetRainbow(lights)

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  init {
    // Configure the button bindings
    configureButtonBindings()
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  fun configureButtonBindings() {
    Constants.loadConstants()
    /* controller0 overrides */
    JoystickButton(controller0, kY.value).whenHeld(FixedIntakeSpeed(intake, { controller0.getY(kLeft) }))
    JoystickButton(controller0, kX.value).whenHeld(FixedIndexerSpeed(indexer, { -controller0.getY(kLeft) }))
    JoystickButton(controller0, kB.value).whenHeld(FixedGateSpeed(gate, { controller0.getY(kLeft) }))
    JoystickButton(controller0, kA.value).whenHeld(FixedShooterSpeed(shooter, { -Constants.kShooterSpeed }))

    val runGate = FixedGateSpeed(gate, { Constants.kGateSpeed })
    val runShooter = FixedShooterSpeed(shooter, { Constants.kShooterSpeed })

    JoystickButton(controller1, kBumperRight.value).and(JoystickButton(controller0, kB.value).negate()).whileActiveContinuous(
            SequentialCommandGroup(
                    runGate.withTimeout(0.5),
                    ParallelCommandGroup(
                            runGate,
                            runShooter
                    ).withTimeout(2.0)
            )
    )

    /* TODO: a button to cancel all active commands and return each subsystem to default command (if things go wrong) */

    /* setup default commands */
    drivetrain.setDefaultCommand(XboxDriveCommand)
    gate.setDefaultCommand(FixedGateSpeed(gate, { 0.0 }))

    indexer.setDefaultCommand(FixedIndexerSpeed(indexer, { -Constants.kIndexerSpeed }))
    intake.setDefaultCommand(FixedIntakeSpeed(intake, { Constants.kIntakeSpeed }))
    lights.setDefaultCommand(setAlliance)

    /* set options for autonomous */
    m_autoCommandChooser.setDefaultOption("Power Port Vision Autonomous", visionHighGoalCommand)
    m_autoCommandChooser.addOption("Backup 2s Autonomous", backupAuto)
    m_autoCommandChooser.addOption("Forward 2s Autonomous", forwardAuto)
    m_autoCommandChooser.addOption("Backup 2s and Look Cool Autonomous", spinAuto)
    m_autoCommandChooser.addOption("No auto (DON'T PICK)", noAuto)

    SmartDashboard.putData("Auto mode", m_autoCommandChooser)

  }

  fun getAutonomousCommand(): Command {
    // Return the selected command
    return m_autoCommandChooser.getSelected()
  }
}
