package frc.robot.subsystems

import edu.wpi.first.wpilibj.SpeedController
import edu.wpi.first.wpilibj2.command.SubsystemBase

class WinchSubsystem(val winchMotor: SpeedController) : SubsystemBase() {
    /**
     * Creates a new ExampleSubsystem.
     */
    init {
    }

    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    override fun periodic() {
        // by default, do not spin
        winchMotor.set(0.0)
    }

    fun setSpeed(speed: Double) {
        // release the winch a certain speed
        winchMotor.set(speed)
    }

    fun getPosition(): Double {
        // TODO
        return 0.0
    }
}
