/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team7200.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Spark;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.CameraServer;

import javax.swing.plaf.basic.BasicInternalFrameTitlePane.RestoreAction;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;

public class Robot extends IterativeRobot {

	Command autoCommand;

	DigitalInput liftswitch = new DigitalInput(0);
	DigitalInput liftDown = new DigitalInput(1);

	SendableChooser<Integer> teamStatus;
	SendableChooser<Integer> autoPlay;
	/****************************************************************************************************/
	protected DifferentialDrive m_myRobot;
	protected Joystick driverstick = new Joystick(0);
	/****************************************************************************************************/
	protected Compressor c = new Compressor(0);
	Solenoid shootSolenoid = new Solenoid(0);
	Solenoid retractSolenoid = new Solenoid(1);
	Solenoid Deploy = new Solenoid(2);
	Solenoid unDeploy = new Solenoid(3);
	/****************************************************************************************************/
	double stickReverse;// multiply by this when opposite control is needed on demand
	/****************************************************************************************************/
	private static I2C Wire = new I2C(Port.kOnboard, 1);// slave I2C device address 1 (rio is master)
	private static I2C Wire1 = new I2C(Port.kOnboard, 2);
	/****************************************************************************************************/
	TalonSRX liftTalon = new TalonSRX(1);

	boolean lifting;
	boolean liftUp;
	boolean lifterDown;

	@Override
	public void robotInit() {

		Spark m_left0 = new Spark(0); // motors are plugged into ports 0,1,2,3 into the roborio
		Spark m_left1 = new Spark(1);
		SpeedControllerGroup m_left = new SpeedControllerGroup(m_left0, m_left1);// motor 0 and 1 are the left side
																					// motors
		Spark m_right2 = new Spark(2);
		Spark m_right3 = new Spark(3);
		SpeedControllerGroup m_right = new SpeedControllerGroup(m_right2, m_right3);// motor 2 and 3 are the right side
																					// motors
		m_myRobot = new DifferentialDrive(m_left, m_right); // new differential drive - another can be created for
															// another set of wheels

		c.setClosedLoopControl(true);// start the compressor

		m_myRobot.arcadeDrive(0, 0);// set drivetrain to 0 movement

	}

	@Override
	public void autonomousInit() {

	}

	@Override
	public void autonomousPeriodic() {

	}

	@Override
	public void teleopInit() {
		CameraServer.getInstance().startAutomaticCapture();// start the camera
	}

	@Override
	public void teleopPeriodic() {

		boolean normalDrive = driverstick.getRawButton(10);
		boolean revDrive = driverstick.getRawButton(12);
		double robotSpeed = driverstick.getThrottle();
		boolean punch = driverstick.getRawButton(2);
		boolean motortest = driverstick.getRawButton(3);
		boolean sendymabob = driverstick.getRawButton(4);
		boolean sendoff = driverstick.getRawButton(6);

		boolean liftInit = driverstick.getRawButton(5);

		shootSolenoid.set(false);
		retractSolenoid.set(true);

		liftTalon.set(ControlMode.PercentOutput, 0);
		liftUp = false;

		m_myRobot.arcadeDrive(driverstick.getX() * robotSpeed * stickReverse, driverstick.getY() * robotSpeed);// drive
																												// the
																												// bot
		/****************************************************************************************************/
		if (liftInit) {
			lifting = !lifting;
		}

		if (liftDown.get()) {
			lifterDown = true;
		}
		if (liftswitch.get()) {
			liftUp = true;
		}

		while (lifting && liftUp == false) {
			liftTalon.set(ControlMode.PercentOutput, 1);
		}

		while (lifting = false && lifterDown) {
			liftTalon.set(ControlMode.PercentOutput, 1);
		}
		/****************************************************************************************************/

		// code for reversing drive direction

		if (revDrive == true) {

			stickReverse = -1.0;
			m_myRobot.arcadeDrive((driverstick.getY() * -1) * 0.7 * stickReverse, (driverstick.getX()) * 0.7);

		}

		if (normalDrive == true) {

			stickReverse = 1;
			m_myRobot.arcadeDrive((driverstick.getY() * -1) * 0.7 * stickReverse, (driverstick.getX()) * 0.7);

		}

		/****************************************************************************************************/

		if (punch == true) { // code for solenoid

			shootSolenoid.set(true);
			retractSolenoid.set(false);

			m_myRobot.arcadeDrive((driverstick.getY() * -1) * 0.7 * stickReverse, (driverstick.getX()) * 0.7);
		}
		/****************************************************************************************************/

		if (motortest == true) {

			liftTalon.set(ControlMode.PercentOutput, 1);
		}
		/****************************************************************************************************/

		if (sendymabob == true) {
			Wire.write(1, 1);

		}
		if (sendoff == true) {
			Wire.write(1, 0);

		}
		/****************************************************************************************************/

	}

	@Override
	public void testPeriodic() {
	}
}
