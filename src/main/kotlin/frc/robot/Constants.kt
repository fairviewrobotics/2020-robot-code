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

    val DrivetrainPID_P = 0.035
    val DrivetrainPID_I = 0.0
    val DrivetrainPID_D = 0.005
    val DrivetrainPID_AngleToleranceDeg = 2.0
    val DrivetrainPID_AngleRateToleranceDegPerS = 1.0


  }
}
