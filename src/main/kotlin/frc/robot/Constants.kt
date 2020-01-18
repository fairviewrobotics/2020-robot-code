/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import frc.robot.commands.TurnToAngle

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.  This class should not be used for any other purpose.  All constants should be
 * declared globally (i.e. public static).  Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
class Constants {
  companion object {
    // Put Constants inside the companion object to make them globally accessible.
    // ex. val motorPort: Int = 0

    val TurnToAngleP = 0.03
    val TurnToAngleI = 0.0
    val TurnToAngleD = 0.01

    val TurnToAngleleranceDeg = 2.0
    val TurnToAngleRateToleranceDegPerS = 1.0

    val subsysP = 0.03
    val subsysI = 0.0
    val subsysD = 0.0
  }
}
