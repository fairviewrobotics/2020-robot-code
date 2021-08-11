package frc.robot.subsystems

import java.time.LocalTime
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
class IntakeSubsystem(val motor0: SpeedController,
                      val motor1: SpeedController,
                      val visionToggle: VisionToggleSubsystem) : SubsystemBase() {
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

    var intakeShutoffDelayStartTime: LocalTime? = null
    val autoShutoffDelay: Long = 1 // seconds after not seeing a ball to keep running automatic intake
    var runningIntakeAutomatically = false

    companion object {
        val ntInst = NetworkTableInstance.getDefault()
        val table = ntInst.getTable("ball-vision")
        val ballHeight = table.getEntry("ballHeight") // the height of the ball
        val ballFound = table.getEntry("ballFound")
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
        val runningBefore = runningIntakeAutomatically
        val shouldRunIntake = (visionToggle.visionIntakeOn
                && ballFound.getBoolean(false)
                && ballHeight.getDouble(0.0) >= Constants.kballHeightForAutoIntake)

        // unless the system is turned off, wait for some time before turning off the intake
        if (!visionToggle.visionIntakeOn) runningIntakeAutomatically = false
        else if (shouldRunIntake) {
            // cancel any delays and start the intake
            intakeShutoffDelayStartTime = null
            runningIntakeAutomatically = true
        } else {
            val now = LocalTime.now()

            // handle losing sight of the ball
            if (runningBefore && !shouldRunIntake) {
                if (intakeShutoffDelayStartTime == null) {
                    // start delay if we haven't already
                    intakeShutoffDelayStartTime = now
                } else if (now.minusSeconds(autoShutoffDelay).isAfter(intakeShutoffDelayStartTime)) {
                    // the delay has passed and we don't see the ball, turn off the automatic intake
                    intakeShutoffDelayStartTime = null
                    runningIntakeAutomatically = false
                }
            }
        }

        return runningIntakeAutomatically
    }
}
