package frc.robot.triggers

import edu.wpi.first.wpilibj2.command.button.Trigger
import edu.wpi.first.wpilibj.DriverStation
import frc.robot.Constants


class EndgameTrigger(): Trigger() {
    val ds = DriverStation.getInstance()

    override fun get(): Boolean {
        /* TODO: check match time
        val currentMatchTime = ds.getMatchTime()
        return currentMatchTime > Constants.kEndgameStart
         */

        return true
    }
}