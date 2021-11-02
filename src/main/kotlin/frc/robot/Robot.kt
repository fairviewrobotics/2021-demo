/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot

import edu.wpi.first.wpilibj.TimedRobot
import edu.wpi.first.wpilibj2.command.Command
import edu.wpi.first.wpilibj2.command.CommandScheduler
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX
import com.kauailabs.navx.frc.AHRS
import com.revrobotics.CANSparkMax
import com.revrobotics.CANSparkMaxLowLevel.MotorType
import com.revrobotics.ColorSensorV3
import edu.wpi.first.wpilibj.*
import edu.wpi.first.wpilibj.GenericHID.Hand.kLeft
import edu.wpi.first.wpilibj.GenericHID.Hand.kRight
import edu.wpi.first.wpilibj.XboxController.Button.*
import edu.wpi.first.wpilibj.drive.DifferentialDrive
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard
import edu.wpi.first.wpilibj2.command.*
import edu.wpi.first.wpilibj2.command.button.*
import frc.robot.commands.*
import frc.robot.subsystems.*
import frc.robot.triggers.EndgameTrigger

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
class Robot : TimedRobot() {
        /* controller0 - secondary driver controller */
        val controller0 = XboxController(1)
        /* controller1 - primary driver controller (overriden by controller0) */
        val controller1 = XboxController(0)
    
        /** --- setup drivetrain --- **/
        val motorFrontLeft = WPI_TalonSRX(Constants.kDrivetrainFrontLeftPort)
        val motorBackLeft = WPI_TalonSRX(Constants.kDrivetrainBackLeftPort)
        val motorFrontRight = WPI_TalonSRX(Constants.kDrivetrainFrontRightPort)
        val motorBackRight = WPI_TalonSRX(Constants.kDrivetrainBackRightPort)
    
    
        /* keep speeds same on motors on each side */
        val motorsLeft = SpeedControllerGroup(motorFrontLeft, motorBackLeft)
        val motorsRight = SpeedControllerGroup(motorFrontRight, motorBackRight)
    
        val visionToggle = VisionToggleSubsystem()
    
        val gyro = AHRS()
        val leftDrivetrainEncoder = Encoder(Constants.leftDrivetrainEncoderPortA, Constants.leftDrivetrainEncoderPortB, Constants.kDrivetrainEncoderAReversed)
        val rightDrivetrainencoder = Encoder(Constants.rightDrivetrainEncoderPortA, Constants.rightDrivetrainEncoderPortB, Constants.kDrivetrainEncoderBReversed)

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    override fun robotInit() {
    }

    /**
     * This function is called periodically during autonomous.
     */
    override fun autonomousPeriodic() {
    }

    /**
     * This function is called periodically during operator control.
     */
    override fun teleopPeriodic() {
    }
}
