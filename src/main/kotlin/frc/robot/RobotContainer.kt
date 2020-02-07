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
import edu.wpi.first.wpilibj.SpeedControllerGroup
import edu.wpi.first.wpilibj.drive.DifferentialDrive

import com.ctre.phoenix.motorcontrol.can.*
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
  // The robot's subsystems and commands are defined here...

  var m_autoCommandChooser: SendableChooser<Command> = SendableChooser()

  val joystick0 = Joystick(0)

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

  /*** --- commands --- ***/
  //drive by a joystick
  val joystickDriveCommand = JoystickDrive(drivetrain, joystick0)

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

  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  init {
    // Configure the button bindings
    configureButtonBindings()
    configureDefaultCommands()
    /* set options for autonomous */
    m_autoCommandChooser.setDefaultOption("Power Port Vision Autonomous", visionHighGoalCommand)
    m_autoCommandChooser.addOption("Backup 2s Autonomous", backupAuto)
    m_autoCommandChooser.addOption("Forward 2s Autonomous", forwardAuto)
    m_autoCommandChooser.addOption("Backup 2s and Look Cool Autonomous", spinAuto)
    m_autoCommandChooser.addOption("No auto (DON'T PICK)", noAuto)

    SmartDashboard.putData("Auto mode", m_autoCommandChooser)

    Constants.loadConstants()
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  fun configureButtonBindings() {
    val alignButton = JoystickButton(joystick0, 4)

    //val turnButton = JoystickButton(joystick0, 4)


    /**
     * TODO: when the vision button is pressed, run a sequential group of commands
     * 1. Drive towards target till it can be seen (DONE)
     * 2. Drive forwards for ~0.5 seconds
     * 3. Shoot
     */
    //alignButton.whenPressed(visionHighGoalCommand)

    //turnButton.whenPressed(turnToAngleCommand)

    JoystickButton(joystick0, 8).whenHeld(FixedIntakeSpeed(intake, { joystick0.getZ() }))
    JoystickButton(joystick0, 9).whenHeld(FixedIntakeSpeed(intake, { -joystick0.getZ() }))

    JoystickButton(joystick0, 11).whenHeld(FixedIndexerSpeed(indexer, { 0.4 }))
    JoystickButton(joystick0, 10).whenHeld(FixedIndexerSpeed(indexer, { -0.4 }))

    JoystickButton(joystick0, 3).whenHeld(FixedGateSpeed(gate, { 0.3 }))
    JoystickButton(joystick0, 2).whenHeld(FixedGateSpeed(gate, { -0.3 }))

    JoystickButton(joystick0, 5).whenHeld(FixedShooterSpeed(shooter, { joystick0.getZ() }))


    /* TODO: a button to cancel all active commands and return each subsystem to default command (if things go wrong) */

  }

  /**
   * Set default commands for each subsystem
   * They will be run if no other commands are sheduled that have a dependency on that subsystem
   */
  fun configureDefaultCommands() {
    drivetrain.setDefaultCommand(joystickDriveCommand)

    indexer.setDefaultCommand(FixedIndexerSpeed(indexer, {0.0}))
    gate.setDefaultCommand(FixedGateSpeed(gate, {0.0}))
  }


  fun getAutonomousCommand(): Command {
    // Return the selected command
    return m_autoCommandChooser.getSelected()
  }
}
