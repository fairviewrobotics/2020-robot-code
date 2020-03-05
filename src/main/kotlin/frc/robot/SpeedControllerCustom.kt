package frc.robot

import edu.wpi.first.wpilibj.SpeedController

class WeightedSpeedController(var coeff: Double, val motor: SpeedController) : SpeedController {
    override fun set(speed: Double) {
        motor.set(speed * coeff)
    }

    override fun get(): Double {
        return motor.get() / coeff
    }

    override fun setInverted(isInverted: Boolean) {
        return motor.setInverted(isInverted)
    }

    override fun getInverted(): Boolean {
        return motor.inverted
    }

    override fun disable() {
        motor.disable()
    }

    override fun stopMotor() {
        motor.stopMotor()
    }

    override fun pidWrite(p0: Double) {
        motor.pidWrite(p0)
    }

}
