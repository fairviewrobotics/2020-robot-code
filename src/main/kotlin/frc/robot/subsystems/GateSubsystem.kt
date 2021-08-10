package frc.robot.subsystems

import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj.util.Color
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import kotlin.math.abs


/**
 * Indexer Subsystem
 *
 * Just a indexer motor
 */
class GateSubsystem(val motor: SpeedController, val colorSensor: ColorSensorV3) : SubsystemBase() {

    override fun periodic() {
        debugMeasurement()
    }

    /* set a motor speed, no matter if a ball is in the gate or not */
    fun setSpeed(speed: Double) {
        motor.set(speed)
    }

    /* set a speed (only run if a ball isn't in gate) */
    fun setSpeedSensored(speed: Double) {
        if (isBallTriggered()) {
            motor.set(0.0)
        } else {
            motor.set(speed)
        }
    }

    /* check if ball sensor is triggered */
    fun isBallTriggered(): Boolean {
        val IR = colorSensor.getIR()
        val empty_thresh = Constants.constants["GateColorSensor_EmptyValue"] ?: 0.0
        val ball_thresh = Constants.constants["GateColorSensor_BallValue"] ?: 50.0

        // report triggered if ir value is closer to ball threshold than empty threshold
        // (or if ir sensor appears to be malfunctioning)
        return IR == 0 || (abs(IR - ball_thresh) <= abs(IR - empty_thresh))
    }

    /* print status of sensor + its reading */
    fun debugMeasurement() {
        SmartDashboard.putBoolean("Gate Ball Sensor Status", colorSensor.getIR() != 0)
        SmartDashboard.putBoolean("Ball Present in Gate", isBallTriggered())
        SmartDashboard.putNumber("Gate Ball Sensor Reading", colorSensor.getIR().toDouble())
    }
}
