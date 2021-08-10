package frc.robot.subsystems

import edu.wpi.first.wpilibj.GenericHID
import edu.wpi.first.wpilibj.XboxController
import edu.wpi.first.wpilibj2.command.CommandBase
import java.time.LocalTime
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import java.lang.IndexOutOfBoundsException
import java.util.*
import kotlin.concurrent.schedule

class DriveModeToggleSubsystem(private val drivetrain: DrivetrainSubsystem,
                               private val controller1: XboxController,
                               private val driveModes: List<CommandBase>,
                               var currentMode: Int) : SubsystemBase() {

    var holdStartTime: LocalTime? = null
    var rumbleStartTime: LocalTime? = null

    override fun periodic() {
    }

    fun incrementMode() {
        currentMode = (currentMode + 1) % driveModes.size

        // rumble the controller
        controller1.setRumble(GenericHID.RumbleType.kLeftRumble, 1.0)
        controller1.setRumble(GenericHID.RumbleType.kRightRumble, 1.0)
        rumbleStartTime = LocalTime.now()
    }

    fun stopRumble() {
        rumbleStartTime = null
        controller1.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0)
        controller1.setRumble(GenericHID.RumbleType.kRightRumble, 0.0)
    }

    fun applyDriveMode() {
        drivetrain.defaultCommand = try {
            driveModes[currentMode]
        } catch (e: IndexOutOfBoundsException) { // this shouldn't happen, but just in case
            driveModes[0]
        }
    }
}