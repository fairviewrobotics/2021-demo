/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands

import frc.robot.subsystems.*
import edu.wpi.first.wpilibj2.command.CommandBase
import frc.robot.Constants

/**
 * Drive the drivetrain based on a joystick
 */
class FixedIndexerSpeed(val indexerSubsystem: IndexerSubsystem, val PDP: PDPSubsystem, val speed: () -> Double) : CommandBase() {
    init {
        addRequirements(indexerSubsystem)
    }

    override fun execute() {
        indexerSubsystem.setSpeed(speed())
        if (PDP.getCurrent(Constants.kIndexerPort)>20.0) {
            indexerSubsystem.setSpeed(-1.0)
        }
    }

    override fun end(interrupted: Boolean) {
        indexerSubsystem.setSpeed(0.0)
    }

    override fun isFinished() = false
}
