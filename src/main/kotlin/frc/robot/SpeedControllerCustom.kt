import com.ctre.phoenix.motorcontrol.ControlMode
import com.ctre.phoenix.motorcontrol.can.*
import edu.wpi.first.wpilibj.SpeedController

class SpeedControllerCustom(var coeff: Double, val motor: SpeedController): 
    SpeedController {
    // Takes speed controller with motor, and coefficient, then just has calls to the motor commands.
    // Occasionally uses the coeff when setting and calculating speed.
    override fun set(speed: Double){
        motor.set(speed*coeff)
    }
    override fun get(): Double{
        if (coeff!=0.0){
            return motor.get()/coeff
        } else{
            return motor.get()
        } 
    }

    fun getCoeff(): Double{
        return coeff
    }

    fun setCoeff(newCoeff: Double){
        coeff = newCoeff
    }

    fun getUnadjusted(): Double{
        return motor.get()
    }

    override fun setInverted(isInverted: Boolean){
        return motor.setInverted(isInverted)
    }

    override fun getInverted():Boolean{
        return motor.getInverted()
    }

    override fun disable(){
        motor.disable()
    }

    override fun stopMotor(){
        motor.stopMotor()
    }

    override fun pidWrite(p0: Double){
        motor.pidWrite(p0)
    }
    
}
