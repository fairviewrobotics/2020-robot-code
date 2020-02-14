

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// An example command. Ideally copy and paste every time you want to create a new command.

package frc.robot.commands

// Import all the required modules: The commands, subsystems, other libraries required in the init
import frc.robot.subsystems.*
import frc.robot.Constants
import edu.wpi.first.wpilibj2.command.CommandBase
import com.revrobotics.CANSparkMax
import com.revrobotics.CANEncoder
import com.revrobotics.CANPIDController
import com.revrobotics.ControlType

class ShooterPID(val shooter: ShooterSubsystem, val targetSpeed: () -> Double) : CommandBase() {
    // In the init, between the parenthesis, include all the subsystems and modules
    // you must include. Optionally, replace CommandBase with PIDCommand, for
    // a PID command
    /**
     * Creates a new ExampleCommand.
     *
     * @param m_subsystem The subsystem used by this command.
     */
    val shooterEncoder = shooter.getEncoder()
    init {

        val pid = shooter.getPidController()
        pid.setP(Constants.constants["ShooterP"]?: 0.05)
        pid.setI(Constants.constants["ShooterI"]?: 0.0)
        pid.setD(Constants.constants["ShooterD"]?: 0.005)
        pid.setFF(Constants.constants["ShooterFF"]?:0.1)
        pid.setReference(targetSpeed(),ControlType.valueOf("kVelocity"))
    }

    // Called every time the scheduler runs while the command is scheduled.
    override fun execute() {
        shooter.pidWrite(shooterEncoder.getVelocity())
    }

    override fun end(interrupted: Boolean) {
        shooter.setSpeed(0.0)
    }

    // Returns true when the command should end.
    override fun isFinished(): Boolean {
        return false
    }
}
