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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class Robot extends IterativeRobot {

	Command autoCommand;
	/****************************************************************************************************/
	DigitalInput l_liftswitch1 = new DigitalInput(0);// elevator limit switch code
	DigitalInput l_liftswitch2 = new DigitalInput(1);// NOTE: if IR sensors are used, True is when the sensor is NOT
														// tripped
	DigitalInput l_liftswitch3 = new DigitalInput(2);
	DigitalInput l_liftDown = new DigitalInput(3);
	/****************************************************************************************************/
	SendableChooser<Integer> teamStatus;
	SendableChooser<Integer> autoPlay;
	/****************************************************************************************************/
	protected DifferentialDrive m_myRobot; // basic driving variables
	protected Joystick driverstick = new Joystick(0);
	/****************************************************************************************************/
	protected Compressor p = new Compressor(0); // pneumatics control variables
	Solenoid p_shootSolenoid = new Solenoid(0);
	Solenoid p_retractSolenoid = new Solenoid(1);
	Solenoid p_Deploy = new Solenoid(2);
	Solenoid p_unDeploy = new Solenoid(3);
	/****************************************************************************************************/
	double stickReverse;// multiply by this when opposite control is needed on demand
	/****************************************************************************************************/
	private static I2C Wire = new I2C(Port.kOnboard, 1);// slave I2C device address 1 (rio is master)
	private static I2C Wire1 = new I2C(Port.kOnboard, 2);
	byte[] i2cbuffer = new byte[8];
	/****************************************************************************************************/
	boolean auto; // auto run variables
	double turnSpeed;
	Ultrasonic s_sensor = new Ultrasonic(4, 5);// ping, then echo
	/****************************************************************************************************/
	TalonSRX m_liftTalon = new TalonSRX(1); // elevator control variables
	private static final int liftDeviceID = 2;
	private CANSparkMax m_liftMotor;

	boolean lifting;
	boolean lift1;
	boolean lift2;
	boolean lift3;
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

		p.setClosedLoopControl(true);// start the compressor

		m_myRobot.arcadeDrive(0, 0);// set drivetrain to 0 movement
		boolean auto = false;
		turnSpeed = 0;
		s_sensor.setAutomaticMode(true);

		m_liftMotor = new CANSparkMax(liftDeviceID, MotorType.kBrushless);
		m_liftMotor.set(0);

	}

	@Override
	public void autonomousInit() {
		turnSpeed = 0;
	}

	@Override
	public void autonomousPeriodic() {

		Wire.read(1, 1, i2cbuffer);
		double servoangle = Math.abs(i2cbuffer[0]);
		double driveAngle = (servoangle - 158) / 45;

		System.out.println(servoangle);

		m_myRobot.arcadeDrive(0.6, turnSpeed);

		turnSpeed = driveAngle;

		if (turnSpeed > 0.6) {
			turnSpeed = 0.6;
		}
		if (turnSpeed < -0.6) {
			turnSpeed = -0.6;
		}
	}

	@Override
	public void teleopInit() {
		CameraServer.getInstance().startAutomaticCapture();// start the camera
		
		turnSpeed = 0;
		m_liftTalon.set(ControlMode.PercentOutput, 0);

		lifting = false;
		lift1 = true;
		lift2 = true;
		lift3 = true;

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
		boolean autoRun = driverstick.getTrigger();
		boolean liftInit = driverstick.getRawButton(5);
		double speed = 0.6;

		double range = s_sensor.getRangeInches();

		p_shootSolenoid.set(false);
		p_retractSolenoid.set(true);

		lift1 = true;
		lift2 = true;
		lift3 = true;
		lifterDown = false;

		m_myRobot.arcadeDrive(driverstick.getY() * robotSpeed * stickReverse, driverstick.getX() * robotSpeed);

		/****************************************************************************************************/
		if (autoRun == true) {

			Wire.read(1, 1, i2cbuffer);

			double servoangle = Math.abs(i2cbuffer[0]);
			double driveAngle = (servoangle - 100) / -30;

			m_myRobot.arcadeDrive(0.6, turnSpeed);

			turnSpeed = driveAngle;

			if (turnSpeed > 0.6) {
				turnSpeed = 0.6;
			}

			if (turnSpeed < -0.6) {
				turnSpeed = -0.6;
			}

		}

		/****************************************************************************************************/
		if (liftInit == true) { // elevator lift operation initialization

			lifting = !lifting;

			m_myRobot.arcadeDrive(driverstick.getY() * robotSpeed * stickReverse, driverstick.getX() * robotSpeed);

			Timer.delay(0.2);

		}

		/****************************************************************************************************/

		if (l_liftDown.get()) { // checking the switches to determine elevator position
			lifterDown = false;
		}

		if (l_liftswitch1.get()) {

			lift1 = false;// AKA no object

		}

		if (l_liftswitch2.get()) {

			lift2 = false;

		}
		if (l_liftswitch3.get()) {

			lift3 = false;

		}

		/****************************************************************************************************/

		if (lifting && lift3 == false) {// if lift operation is initialized and level 3 not reached

			m_liftTalon.set(ControlMode.PercentOutput, 1);// wind er up
		}

		if (lifting == false && lifterDown == false) {// if lift button is pushed again, and the lifter has not down

			m_liftTalon.set(ControlMode.PercentOutput, -1);
		}

		if (lifting == false && lifterDown) {

			m_liftTalon.set(ControlMode.PercentOutput, 0);
		}

		if (lifting && lift3) {

			m_liftTalon.set(ControlMode.PercentOutput, 0);
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

			p_shootSolenoid.set(true);
			p_retractSolenoid.set(false);

			m_myRobot.arcadeDrive((driverstick.getY() * -1) * 0.7 * stickReverse, (driverstick.getX()) * 0.7);
		}
		/****************************************************************************************************/

		if (motortest == true) {

			m_liftMotor.set(0.5);
		}

		if (driverstick.getRawButton(11)) {
			m_liftMotor.set(0);
		}
		/****************************************************************************************************/

		if (sendymabob == true) {  // writing stuff to i2c address 1 arduino
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
