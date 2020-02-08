package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer
import edu.wpi.first.wpilibj.DriverStation

class LEDSubsystem(val m_led: AddressableLED, val m_len: Int, val driverstation: DriverStation): SubsystemBase() {

  val m_ledBuffer = AddressableLEDBuffer(m_len)
  val length = m_ledBuffer.getLength()

  init {
    m_led.setLength(m_ledBuffer.getLength())
    m_led.setData(m_ledBuffer)
    m_led.start()
  }

  //Default command: Lights off
  override fun periodic() {
    for (i in 0 until m_ledBuffer.getLength()) {

      m_ledBuffer.setRGB(i, 0, 0, 0)
   }
   m_led.setData(m_ledBuffer)
  }

  //For setting light using RGB color
  fun setRGB(i: Int, r: Int, g: Int, b: Int) {

      m_ledBuffer.setRGB(i, r, g, b)

  }

  //For setting light using HSV color
  fun setHSV(i: Int, h: Int, s: Int, v: Int) {

    m_ledBuffer.setHSV(i, h, s, v)

}
//For displaying the buffered color(s) on the lights
  fun display() {
    m_led.setData(m_ledBuffer)
  }

  //Returns length of the light string
  fun returnLength(): Int {
    return length
  }

  //Determines the alliance of the robot
  fun returnAlliance(): String {

    var color = driverstation.getAlliance()

    if(color == DriverStation.Alliance.Blue){
      return "Blue"
    }
    else if (color == DriverStation.Alliance.Red){
      return "Red"
    } 
    else{
      return "None"
  }
}
}