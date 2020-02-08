package frc.robot.subsystems

import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.AddressableLED
import edu.wpi.first.wpilibj.AddressableLEDBuffer

class LEDSubsystem(val m_led: AddressableLED, val m_len: Int) : SubsystemBase() {

  val m_ledBuffer = AddressableLEDBuffer(m_len)
  val length = m_ledBuffer.getLength()

  init {
    m_led.setLength(m_ledBuffer.getLength())
    m_led.setData(m_ledBuffer)
    m_led.start()
  }

  //Default command: Lights white
  override fun periodic() {
    for (i in 0 until m_ledBuffer.getLength()) {

      m_ledBuffer.setRGB(i, 0, 0, 0)
   }
   m_led.setData(m_ledBuffer)
  }

  fun setRGB(i: Int, r: Int, g: Int, b: Int) {

      m_ledBuffer.setRGB(i, r, g, b)

  }

  fun setHSV(i: Int, h: Int, s: Int, v: Int) {

    m_ledBuffer.setHSV(i, h, s, v)

}

  fun display() {
    m_led.setData(m_ledBuffer)
  }

  fun returnLength(): Int {
    return length
  }
}
