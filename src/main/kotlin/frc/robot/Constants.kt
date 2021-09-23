/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import edu.wpi.first.networktables.EntryListenerFlags
import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.kinematics.*

/**
 * Constants Managment:
 *
 * Constants are stored in a hashmap
 * On robotinit, constants are loaded from persistent networktable entries
 * If no saved entry is found, a hardcoded default is used
 * code using constants may register listeners for any constant changes (useful for tuning parameters at runtime, like pid loops)
 */

private data class ConstantsListener(val func: () -> Unit, val id: Int)

class Constants {
    companion object {
        val kShooterDir = -1.0
        val kGateDir = 1.0
        val kIntakeDir = 1.0
        val kIntake2Dir = -1.0
        val kIndexerDir = 1.0
        val kWinchDir = 1.0
        val kClimberDir = 1.0

        val kShooterSpeed = 1.0 * kShooterDir
        val kShooterReverseSpeed = -0.3 * kShooterDir
        val kGateSpeed = 1.0 * kGateDir
        val kGateRSpeed = -0.5 * kGateDir
        val kGateRDelay = 0.1
        val kGateLoadSpeed = 0.5 * kGateDir
        val kIntakeSpeed = 1.0 * kIntakeDir
        val kIntake2Speed = 0.85
        val kIndexerSpeed = 0.75 * kIndexerDir
        val kWinchSpeed = 1.0 * kWinchDir
        val kClimberSpeed = 0.75 * kClimberDir
        val kClimberDownSpeed = 0.3 * kClimberDir

        val kClimberTriggerThresh = 0.15
        val kTriggerThresh = 0.15

        val kWinchPort = 12
        val kClimberPort = 6

        val kGatePort = 2
        val kIndexerPort = 3
        val kIntakePort = 4
        val kIntake2Port = 11
        val kShooterPort = 10
        val kLED0Port = 0

        val kDrivetrainFrontLeftPort = 5
        val kDrivetrainBackLeftPort = 7
        val kDrivetrainFrontRightPort = 8
        val kDrivetrainBackRightPort = 9
        /*val kDrivetrainFrontLeftPort = 4
        val kDrivetrainBackLeftPort = 11
        val kDrivetrainFrontRightPort = 2
        val kDrivetrainBackRightPort = 1*/

        val kEndgameStart = 120

        /* time to shoot three balls */
        val kAutoShootTime = 2.5

        val kAutoBackupTime = 1.0

        /* time for shooter to speed up */
        val kShooterSpinUpTime = 1.0
        /* wavelength of gate pulsing */
        val kGatePulseTime = 0.4
	val kGatePulseWidth = 0.44

        /* path following ****UNTUNED**** */

        /* encoder ports */
        val leftDrivetrainEncoderPortA = 0
        val leftDrivetrainEncoderPortB = 1
        val rightDrivetrainEncoderPortA = 2
        val rightDrivetrainEncoderPortB = 3

        val kDrivetrainEncoderAReversed = false
        val kDrivetrainEncoderBReversed = false
        /* encoder characteristics */
        val ENCODER_EDGES_PER_REV = 360.0
        val WHEEL_DIAMETER = 0.1524
        val kDrivetrainEncoderDistancePerPulse = (1.0 / ENCODER_EDGES_PER_REV) * WHEEL_DIAMETER * Math.PI
        val kGyroReversed = -1.0

        /* feedforward pathfollowing gains */
        val ksVolts = 1.0
        val kvVoltSecondsPerMeter = 2.42
        //val kaVoltSecondsSquaredPerMeter = 0.0000095
         val kaVoltSecondsSquaredPerMeter = 0.05
        /* feedback pathfollowing gains */
        val kPDriveVel = 5.0

        /* max speed and acceleration for pathfollowing */
        val kMaxSpeedMetersPerSecond = 1.0
        val kMaxAccelerationMetersPerSecondSquared = 1.0

        /* pathfollowing ramsete gains */
        val kRamseteB = 2.0
        val kRamseteZeta = 0.7

        /* pathfollowing odeometry */
        /* width between wheels */
        val kTrackwidthMeters = 0.558
        val kDriveKinematics = DifferentialDriveKinematics(kTrackwidthMeters)

        val kballHeightForAutoIntake = 0.33 // a third of the screen
        val kBallAreaThreshold = 0.1

        val kDriveToggleHoldTime = 5 // seconds to hold button to change drive mode
        val kDriveModeChangeRumbleTime = 1
        val kDriveModeChangingOn = true // turn this off to prevent changing

        val constants = mutableMapOf(
            "DrivetrainPID_P" to 0.035,
            "DrivetrainPID_I" to 0.0,
            "DrivetrainPID_D" to 0.005,
            "DrivetrainPID_AngleToleranceDeg" to 2.0,
            "DrivetrainPID_AngleRateToleranceDegPerS" to 1.0,
            "DrivetrainSpeedCoeff_Left" to 1.0,
            "DrivetrainSpeedCoeff_Right" to 1.0,
            "WinchPID_P" to 0.035,
            "WinchPID_I" to 0.0,
            "WinchPID_D" to 0.005,
            "WinchPID_PositionTolerance" to 1.0,
            "GateColorSensor_EmptyValue" to 0.0,
            "GateColorSensor_BallValue" to 50.0
        )

        /** NetworkTables Constants Management **/
        private lateinit var table: NetworkTable
        private var listeners = mutableListOf<ConstantsListener>()
        private var listenersId = 0

        /**
         * Load constants from network tables
         */
        fun loadConstants() {
            table = NetworkTableInstance.getDefault().getTable("Preferences")
            for (key in constants.keys) {
                if (constants[key] == null) continue

                val entry = table.getEntry(key)
                entry.setDefaultDouble(constants[key] ?: 0.0)
                entry.setPersistent()
                constants[key] = entry.getDouble(constants[key] ?: 0.0)

                entry.addListener({ event ->
                    onNetworkTablesChange()
                }, EntryListenerFlags.kNew or EntryListenerFlags.kUpdate)
            }
        }

        private fun onNetworkTablesChange() {
            /** update map **/
            for (key in constants.keys) {
                val entry = table.getEntry(key)
                constants[key] = entry.getDouble(constants[key] ?: 0.0)
            }
            listeners.map { listen -> listen.func() }
        }

        /**
         * Add a listener for any changes in the Preferences NetworkTable
         * listeners will be called on any change
         * Return an id for the listener that can latter be passed to removeListener
         */
        fun addListener(listener: () -> Unit): Int {
            val id = listenersId++
            listeners.add(ConstantsListener(listener, id))
            return id
        }

        /**
         * Remove a listener with an id created by a call to AddListener
         */
        fun removeListener(id: Int) {
            listeners.removeIf { listen -> listen.id == id }
        }
    }
}
