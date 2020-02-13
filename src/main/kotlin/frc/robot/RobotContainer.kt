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
import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.Joystick
import edu.wpi.first.wpilibj.SpeedControllerGroup
import edu.wpi.first.wpilibj.drive.DifferentialDrive

import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.*
import com.kauailabs.navx.frc.AHRS
import edu.wpi.first.wpilibj2.command.button.JoystickButton
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard
import edu.wpi.first.networktables.NetworkTableEntry
import edu.wpi.first.networktables.EntryNotification
import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTableInstance
/**
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private val m_exampleSubsystem: ExampleSubsystem = ExampleSubsystem()

  val m_autoCommand: ExampleCommand = ExampleCommand(m_exampleSubsystem)

  var m_autoCommandChooser: SendableChooser<Command> = SendableChooser()

  val joystick0 = Joystick(0)

  /** --- setup drivetrain --- **/
  val motorFrontLeft =  WPI_TalonSRX(2)
  val motorBackLeft =   WPI_TalonSRX(1)
  val motorFrontRight = WPI_TalonSRX(4)
  val motorBackRight =  WPI_TalonSRX(3)

  /* keep speeds same on motors on each side */
  val motorsLeft = SpeedControllerGroup(motorFrontLeft, motorBackLeft)
  val motorsRight = SpeedControllerGroup(motorFrontRight, motorBackRight)

  val gyro = AHRS()

  val drivetrain = DrivetrainSubsystem(DifferentialDrive(motorsLeft, motorsRight), gyro)

  val joystickDriveCommand = JoystickDrive(drivetrain, joystick0)

  val main = Shuffleboard.getTab("Main") // TODO: Rename this sometime.
  val knockout = Shuffleboard.getTab("Auto mode")
  var speed = 0 // leo: for a worthless experiment :)

  var knockFrontLeft: NetworkTableEntry = knockout.add("Knockout FrontLeft",false).withWidget("Toggle Button").getEntry()
  var knockBackLeft: NetworkTableEntry = knockout.add("Knockout BackLeft",false).withWidget("Toggle Button").getEntry()
  var knockFrontRight: NetworkTableEntry = knockout.add("Knockout FrontRight",false).withWidget("Toggle Button").getEntry()
  var knockBackRight: NetworkTableEntry = knockout.add("Knockout BackRight",false).withWidget("Toggle Button").getEntry()


  /**
   * The container for the robot.  Contains subsystems, OI devices, and commands.
   */
  init {
    // Configure the button bindings
    configureButtonBindings()
    configureDefaultCommands()
    m_autoCommandChooser.setDefaultOption("Default Auto", m_autoCommand)
    Shuffleboard.selectTab("Main")
    main.add("Auto mode", m_autoCommandChooser)
    
    
  }

  /**
   * Use this method to define your button->command mappings.  Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling passing it to a
   * {@link edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  fun configureButtonBindings() {
    val alignButton = JoystickButton(joystick0, 3)

    alignButton.whenPressed(TurnToAngle(drivetrain, 90.0))
    
    // Knockout Listeners
    knockFrontLeft.addListener({event ->
       if(knockFrontLeft.getBoolean(false)) {
         motorFrontLeft.disable()
       }
       // Adds listener, if knockFrontLeft is changed from default false it will disable (hence the event anonymous function)
      }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate) // Uses the listener events update or new for the event.

      // The following code uses the same logic.
    
    knockFrontRight.addListener({event ->
        if(knockFrontRight.getBoolean(false)){
          motorFrontRight.disable()
        }
       }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
    
    knockBackLeft.addListener({event ->
        if(knockBackLeft.getBoolean(false)){
          motorBackLeft.disable()
        }
       }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
    
    knockBackRight.addListener({event ->
        if(knockBackRight.getBoolean(false)){
          motorBackRight.disable()
        }
       }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
    //val shiftUpButton = JoystickButton(joystick0, 2)
    //val shiftDownButton = JoystickButton(joystick0, 4)

    //shiftUpButton.whenPressed(speed++10)
    //shiftDownButton.whenPressed(speed--10)
  }

  /**
   * Set default commands for each subsystem
   * They will be run if no other commands are sheduled that have a dependency on that subsystem
   */
  fun configureDefaultCommands() {
    drivetrain.setDefaultCommand(joystickDriveCommand);
  }


  fun getAutonomousCommand(): Command {
    // Return the selected command
    return m_autoCommandChooser.getSelected()
  }
}
