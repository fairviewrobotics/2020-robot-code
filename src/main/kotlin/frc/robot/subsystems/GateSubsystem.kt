package frc.robot.subsystems

import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.util.Color


/**
 * Indexer Subsystem
 *
 * Just a indexer motor
 */
class GateSubsystem(val motor: SpeedController, val colorSensor: ColorSensorV3) : SubsystemBase() {
    /* default command (don't spin) */
    override fun periodic() {
        setSpeed(0.0)
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        motor.set(speed)

        val detectedColor: Color = colorSensor.getColor()

        val IR: Int = colorSensor.getIR()

        SmartDashboard.putNumber("Red", detectedColor.red)
        SmartDashboard.putNumber("Green", detectedColor.green)
        SmartDashboard.putNumber("Blue", detectedColor.blue)
        SmartDashboard.putNumber("IR", IR.toDouble())

    }
}
