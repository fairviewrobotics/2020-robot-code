package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.SpeedController
import frc.robot.Constants

/**
 * Intake Subsystem
 *
 * Just a intake motor
 */
class IntakeSubsystem(val motor: SpeedController) : SubsystemBase() {
    /* get a nice speed curve
     * do each speed transition across 20 iterations (400ms)
     */

    var pSpeed = 0.0
    var curveCounter = 0
    var isCurve = false
    var internalTargetSpeed = 0.0
    var speedIncPerT = 0.0

    val curveLen = 200

    var internalSpeed = 0.0


    private fun setTargetSpeed(target: Double) {
        internalTargetSpeed = target

        if(internalTargetSpeed != pSpeed) {
            isCurve = true
            curveCounter = 0

            speedIncPerT = (internalTargetSpeed - internalSpeed) / curveLen
        }

        pSpeed = internalTargetSpeed
    }

    private fun updateSpeedCurve() {
        if(isCurve) {
            internalSpeed += speedIncPerT
            curveCounter++

            if(curveCounter >= curveLen) isCurve = false
        }

        motor.set(internalSpeed)
    }

    /* default command (don't spin) */
    override fun periodic() {
        //motor.set(Constants.constants["IntakeBaseSpeed"] ?: 0.0)
        setTargetSpeed(0.0)
        updateSpeedCurve()
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        setTargetSpeed(speed)
        updateSpeedCurve()
    }
}
