/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems // Makes the code importable.

import edu.wpi.first.wpilibj.DigitalOutput
import edu.wpi.first.wpilibj2.command.SubsystemBase
import edu.wpi.first.wpilibj.PowerDistributionPanel

class PDPSubsystem(val PDP: PowerDistributionPanel) : SubsystemBase() {
    /**
     * Creates a new ExampleSubsystem.
     */
    init {
    }

    fun getVoltage(): Double{
        return PDP.getVoltage()
    }

    fun getTemperature(): Double{
        return PDP.getTemperature()
    }

    fun getTotalCurrent(): Double{
        return PDP.getTotalCurrent()
    }

    fun getTotalEnergy(): Double{
        return PDP.getTotalEnergy()
    }

    fun getCurrent(port:Int): Double{
        return PDP.getCurrent(port)
    }
    /**
     * Will be called periodically whenever the CommandScheduler runs.
     */
    override fun periodic() {
    }
}
