package frc.robot.subsystems

import edu.wpi.first.networktables.NetworkTable
import edu.wpi.first.networktables.NetworkTableInstance
import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj2.command.SubsystemBase
import frc.robot.Constants
import frc.robot.commands.BallVision

/**
 * Intake Subsystem
 *
 * Just a intake motor
 */
class IntakeSubsystem(val motor0: SpeedController, val motor1: SpeedController) : SubsystemBase() {
    /* get a nice speed curve
     * do each speed transition across 20 iterations (400ms)
     */

    var pSpeed = 0.0
    var curveCounter = 0
    var isCurve = false
    var internalTargetSpeed = 0.0
    var speedIncPerT = 0.0

    val curveLen = 30

    var internalSpeed = 0.0

    companion object {
        val ntInst = NetworkTableInstance.getDefault()
        val table = BallVision.ntInst.getTable("ball-vision")
        val ballHeight = BallVision.table.getEntry("ballHeight") // the height of the ball
    }


    private fun setTargetSpeed(target: Double) {
        internalTargetSpeed = target

        if (internalTargetSpeed != pSpeed) {
            isCurve = true
            curveCounter = 0

            speedIncPerT = (internalTargetSpeed - internalSpeed) / curveLen
        }

        pSpeed = internalTargetSpeed
    }

    private fun updateSpeedCurve() {
        if (isCurve) {
            internalSpeed += speedIncPerT
            curveCounter++

            if (curveCounter >= curveLen) isCurve = false
        }

        motor0.set(internalSpeed)
        motor1.set(internalSpeed * Constants.kIntake2Dir * Constants.kIntake2Speed)
    }

    override fun periodic() {
        updateSpeedCurve()
    }

    /* set a motor speed */
    fun setSpeed(speed: Double) {
        setTargetSpeed(speed)
    }

    fun getRunningAutomatic(): Boolean {
        return ballHeight.getDouble(0.0) >= Constants.ballHeightForAutoIntake
    }
}
