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
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.buttons.Trigger;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Ultrasonic;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.InetSocketAddress;
import java.net.UnknownHostException;
import java.nio.ByteBuffer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableEntry;

public class Robot extends IterativeRobot {

	Command autoCommand;// have no idea what this does
	double rpi_Turn;// line tracking with raspberry pi vars
	NetworkTableEntry rpi_TurnE;// raspberry stuff
	/****************************************************************************************************/
	DigitalInput b_ballIn = new DigitalInput(2);// check for ball in bucket
	DigitalInput limitUp = new DigitalInput(3);// top limit switch
	DigitalInput limitDown = new DigitalInput(5);// bottom limit switch
	/****************************************************************************************************/
	SendableChooser<Integer> teamStatus;// smart dasboard stuff
	SendableChooser<Integer> autoPlay;
	/****************************************************************************************************/
	protected DifferentialDrive m_myRobot; // basic driving variables
	protected Joystick driverstick = new Joystick(0);// main driving joystick
	protected Joystick techstick = new Joystick(1);// copilot stick
	/****************************************************************************************************/
	protected Compressor p = new Compressor(0); // pneumatics control variables
	Solenoid p_shootSolenoid = new Solenoid(6);
	Solenoid p_retractSolenoid = new Solenoid(7);
	Solenoid p_Deploy = new Solenoid(4);
	Solenoid p_unDeploy = new Solenoid(5);
	/****************************************************************************************************/
	double stickReverse;// multiply by this when opposite control is needed on demand
	/****************************************************************************************************/
	private static I2C Wire = new I2C(Port.kOnboard, 1);// slave I2C device address 1 (rio is master)
	private static I2C Wire1 = new I2C(Port.kOnboard, 2);
	byte[] i2cbuffer = new byte[8];
	/****************************************************************************************************/
	boolean auto; // auto run variables
	double turnSpeed;
	Ultrasonic s_sensor = new Ultrasonic(0, 1);// ping, then echo
	/****************************************************************************************************/
	private static final int liftDeviceID = 0;// elevator BRUSHLESS motor
	private CANSparkMax m_liftMotor;
	private CANEncoder m_encoder;
	/****************************************************************************************************/
	TalonSRX m_eject = new TalonSRX(3); // creating motor controller objects
	VictorSPX m_ballIn = new VictorSPX(2);
	TalonSRX m_tilt = new TalonSRX(1);
	/****************************************************************************************************/
	boolean ballIn; // switches when there is a ball in the bot
	boolean lifttopMax;// used in conjunction with elevator limit switches
	boolean liftdownMin;
	boolean levelQue;// used to prevent switching twice of level requests
	boolean downRequest;// these are like switches to trigger elevator motion
	boolean levelReq;
	boolean level2Req;

	Timer popTime = new Timer();// provides spool time for the top motor
	Timer timer = new Timer();// provides a delay to prevent pistons from deploying too early
	boolean timerstarted;// prevents constant restarting timer
	boolean testVar;// for testing purposes only
	boolean shootswitch;// changes the power of ejecting the ball
	boolean donotDeploy;// prevents the pistons from accidentally deploying
	boolean crazyMode; // allows demo operation
	boolean targetLock;

	/****************************************************************************************************/

	@Override
	public void robotInit() {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();// raspberry stuff
		NetworkTable table = inst.getTable("datatable");
		rpi_TurnE = table.getEntry("angle");

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
		turnSpeed = 0;
		s_sensor.setAutomaticMode(true);// set the distance rangefinder to AUTOMATIC mode

		m_liftMotor = new CANSparkMax(liftDeviceID, MotorType.kBrushless);// new brushless motor object for elevator
		m_encoder = m_liftMotor.getEncoder();// defining the encoder
		m_liftMotor.set(0);// initially sets the motor to stop

		m_ballIn.set(ControlMode.PercentOutput, 0);// initially set the ball handling motors to stop
		m_eject.set(ControlMode.PercentOutput, 0);

		levelQue = false;// setting other variables to inital values to start off with
		levelReq = false;
		level2Req = false;
		downRequest = false;
		timerstarted = false;
		testVar = false;
		shootswitch = true;
		donotDeploy = false;
		crazyMode = false;
		targetLock = false;
	}

	@Override
	public void autonomousInit() {
	}

	@Override
	public void autonomousPeriodic() {

		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("datatable");
		rpi_TurnE = table.getEntry("angle");
		rpi_Turn = rpi_TurnE.getDouble(0);
		System.out.println(rpi_Turn);

		// auto ball pick up - this needs tweaking

		// if(rpi_Turn)
		if (driverstick.getTrigger()) {
			int reverse = 1;
			double turnspeed = 0.4;
			if (rpi_Turn == 0) {
				m_myRobot.arcadeDrive(0, 0);
			}
			if (rpi_Turn == 1) {
				m_myRobot.arcadeDrive(0.5, reverse * 1 * turnspeed);
			}
			if (rpi_Turn == 2) {
				m_myRobot.arcadeDrive(0.5, 0);
			}
			if (rpi_Turn == 3) {
				m_myRobot.arcadeDrive(0.5, reverse * -1 * turnspeed);
			}
		} else {
			m_myRobot.arcadeDrive(0, 0);
		}

	}

	@Override
	public void teleopInit() {
		CameraServer.getInstance().startAutomaticCapture();// start the camera

		turnSpeed = 0;

	}

	@Override
	public void teleopPeriodic() {

		// rpi_TurnE.setDouble(rpi_Turn);
		// notes - configure spark max and talon and victor spx CANOPEN addresses

		String elevatorStatus = "Moving";// anything with qoutes is stuff to print on smartdashboard
		String bucket = "Ball";
		String requestPosition = "error, see Kayden or Alexei for more info";
		String targetDetect = "ball not detected";
		boolean ballDetect = false;

		boolean normalDrive = driverstick.getRawButton(10); // declaring what the joystick buttons are
		boolean revDrive = driverstick.getRawButton(12);
		double robotSpeed = (driverstick.getThrottle() - 1.0) / -2;// speed control of the robot
		boolean autoRun = driverstick.getRawButton(2);
		boolean autoRund = techstick.getRawButton(2);

		boolean level1 = driverstick.getRawButton(3);
		boolean level1d = techstick.getRawButton(3);
		boolean level2 = driverstick.getRawButton(4);
		boolean level2d = techstick.getRawButton(4);
		boolean down = driverstick.getRawButton(5);
		boolean downd = techstick.getRawButton(5);

		boolean autoLockout = driverstick.getTrigger();
		boolean techLock = techstick.getRawButton(9);
		crazyMode = techstick.getRawButton(12);

		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("datatable");
		rpi_TurnE = table.getEntry("angle");
		rpi_Turn = rpi_TurnE.getDouble(0);

		if ((level1 || level1d) && !levelQue) {// if button 3 on either joystick is pressed and has not been held down
			levelQue = true;// request elevator to move to level1
			level2Req = false;// do not go to level2
			downRequest = false;// do not go down
			levelReq = true;// just qued an elevator height, sets this varible to true to prevent switching
							// until button is released

		}
		if ((level2 || level2d) && !levelQue) {// same idea, but for level 2
			levelQue = true;
			levelReq = false;
			downRequest = false;
			level2Req = true;

		}

		if ((down || downd) && !levelQue) {// for lowering elevator
			levelQue = true;
			levelReq = false;
			level2Req = false;
			downRequest = true;

		}
		if (!level1 && !level2 && !down) {// if no buttons have been pressed, allows switching of elevator variables
			levelQue = false;
		}

		double range = s_sensor.getRangeInches();// range is thedistance that the sensor gets

		Wire.read(1, 2, i2cbuffer);

		if (i2cbuffer[1] == 1) {
			targetLock = false;
			targetDetect = "ball in front of bot, enable auto for pickup";
			ballDetect = true;
		}
		if (rpi_Turn != 0) {
			targetLock = true;
			targetDetect = "line detected, auto may be activated";
			ballDetect = false;
		}
		if (rpi_Turn == 0 && i2cbuffer[1] == 0) {
			targetLock = false;
			ballDetect = false;
			targetDetect = "no object detected, auto will not function";
		}
		// System.out.println(range);
		m_liftMotor.set(0);

		m_myRobot.arcadeDrive(driverstick.getY() * robotSpeed * stickReverse, driverstick.getX() * robotSpeed);// drives
																												// the
																												// robot

		if (techstick.getRawButton(10)) {
			Wire.write(1, 1);// turns leds on
		} else {
			Wire.write(1, 0);// turns leds off
		}

		/****************************************************************************************************/
		// checking the switches
		if (b_ballIn.get()) {// if there is a signal from the switch, sets the ball variable to false

			ballIn = false;// AKA no object in bucket
			bucket = "No ball detected, elevator will rise to disk levels";

		} else {

			ballIn = true;
			bucket = "ball detected, elevator will rise to ball levels";
		}

		if (limitUp.get()) {

			lifttopMax = true;

		}

		else {

			lifttopMax = false;
		}

		if (limitDown.get()) {

			liftdownMin = true;
		}

		else {

			liftdownMin = false;
		}
		// ATTENTION - MAKE CODE TO WRITE TO SMARTDASHBOARD WHAT IS DETECTED
		/****************************************************************************************************/
		if (autoRun || autoRund) { // auto ball pick up - if the auto mode is enabled

			if (ballDetect) {

				Wire.read(1, 1, i2cbuffer);// read the pixy values
				m_ballIn.set(ControlMode.PercentOutput, -1.0);

				double servoangle = (i2cbuffer[0]);
				double driveAngle = (servoangle) / 20;

				m_myRobot.arcadeDrive(0.6, turnSpeed);// drive the robot

				turnSpeed = driveAngle;

				if (turnSpeed > 0.6) {
					turnSpeed = 0.6;
				}

				if (turnSpeed < -0.6) {
					turnSpeed = -0.6;
				}

				if (!ballIn) {// if there is no ball in the bucket, activate appropriate motors to pick up

					if (range <= 40) {
						m_myRobot.arcadeDrive(0.6, turnSpeed);

						m_ballIn.set(ControlMode.PercentOutput, -1.0);
						// m_eject.set(ControlMode.PercentOutput, 0.2);
					}

					else {

						m_myRobot.arcadeDrive(0.6, turnSpeed);

						m_ballIn.set(ControlMode.PercentOutput, -1.0);
						m_eject.set(ControlMode.PercentOutput, 0);
					}

				}
			}

			if (targetLock) {

				int reverse = 1;
				double turnspeed = 0.5;
				if (rpi_Turn == 0) {
					m_myRobot.arcadeDrive(0, 0);
				}
				if (rpi_Turn == 1) {
					m_myRobot.arcadeDrive(0.5, reverse * 1 * turnspeed);
				}
				if (rpi_Turn == 2) {
					m_myRobot.arcadeDrive(0.5, 0);
				}
				if (rpi_Turn == 3) {
					m_myRobot.arcadeDrive(0.5, reverse * -1 * turnspeed);
				}
			}

		}

		/****************************************************************************************************/
		if ((driverstick.getPOV() != -1) || (techstick.getPOV() != -1)) {// if hat switch is pushed - this section may
																			// be
																			// confusing the pov switch on
			// the joystick returns a value of -1 if not pressed

			if (driverstick.getPOV() == 180 || techstick.getPOV() == 180) {// if its pushed back, suck the ball in
				m_ballIn.set(ControlMode.PercentOutput, -0.8);// runs the motor to pull the ball in
			}

			if ((driverstick.getPOV() == 0 || techstick.getPOV() == 0) && !ballIn && timer.get() >= 2) {// if there is
																										// no ball and
																										// the switch is
				// pushed forward and more than 5 seconds
				// have passed since there was a ball in the
				// robot
				p_shootSolenoid.set(true);
				p_retractSolenoid.set(false);
			}

			if (((driverstick.getPOV() == 0 || techstick.getPOV() == 0) && ballIn)) {// if the pov switch is pushed
																						// forward and there is a ball
																						// in the
				// bot

				if (!timerstarted) {// if the timer has not already started

					popTime.start();// start the timer

				}

				timerstarted = true;// ensure that the timer does not start a second time

				if (shootswitch && !crazyMode) {// if the piston angle is down

					m_eject.set(ControlMode.PercentOutput, -0.2);// start spooling the motor up

					if (popTime.get() >= 0.5) {// spool for 0.5 seconds

						m_eject.set(ControlMode.PercentOutput, -0.5);// after 0.5 seconds feed the ball into the
																		// spinning motor with the lower motor
						m_ballIn.set(ControlMode.PercentOutput, -1.0);
						timer.reset();
						timer.start();

					}

					else {

						m_ballIn.set(ControlMode.PercentOutput, 0);// if enough time has not passed do not feed the ball
																	// in

					}
				}

				else { // if pistons are up, same idea as above, just spools the motors faster and
						// longer

					m_eject.set(ControlMode.PercentOutput, -0.7);

					if (popTime.get() >= 1.0) {

						m_eject.set(ControlMode.PercentOutput, -0.8);
						m_ballIn.set(ControlMode.PercentOutput, -1.0);
						timer.reset();
						timer.start();

					}

					else {

						m_ballIn.set(ControlMode.PercentOutput, 0);

					}

				}
			}

		}

		else {// if the pov switch is not pressed
			testVar = false;
			timerstarted = false;
			popTime.reset();// reset and stop the timer
			popTime.stop();

			p_shootSolenoid.set(false);
			p_retractSolenoid.set(true);

			m_ballIn.set(ControlMode.PercentOutput, 0);
			m_eject.set(ControlMode.PercentOutput, 0);
		}
		/****************************************************************************************************/

		if (revDrive) {// if the button to flip the controls is pressed

			stickReverse = -1.0;// reverse the driving direction
			m_myRobot.arcadeDrive((driverstick.getY() * -1) * 0.7 * stickReverse, (driverstick.getX()) * 0.7);

		}

		if (normalDrive) {// if the button to reset controls to normal is pressed

			stickReverse = 1;// do not reverse the driving direction
			m_myRobot.arcadeDrive((driverstick.getY() * -1) * 0.7 * stickReverse, (driverstick.getX()) * 0.7);

		}

		/****************************************************************************************************/
		if (driverstick.getRawButton(7) || techstick.getRawButton(7)) {// if button 7 is pressed, lower the piston to
																		// change the shooting angle
			p_Deploy.set(true);
			p_unDeploy.set(false);
			shootswitch = false;
		}
		if (driverstick.getRawButton(8) || techstick.getRawButton(8)) {// if button 8 is pressed, raise the pistons
			p_Deploy.set(false);
			p_unDeploy.set(true);
			shootswitch = true;
		}
		/****************************************************************************************************/
		if (level1d && techLock) {
			m_liftMotor.set(1.0);// if auto mode override, manually raise the elevator
		}

		if (downd && techLock) {// if auto mode override, manually lower the elevator
			m_liftMotor.set(-0.5);
		}
		if (techLock && !level1d && !downd) {// if there is manual override stop the motor if nothing else is selected
			m_liftMotor.set(0);
		}

		/****************************************************************************************************/
		if (ballIn && !autoLockout && !techLock && m_encoder.getPosition() <= 510 && !lifttopMax) {// if manual override is not activated and the bot has a ball

			if (levelReq && !lifttopMax) {// if level1 requested and the motor has not yet got to
											// position 250

				double liftspeed = (m_encoder.getPosition() - 250) / -20;// this sets the lift speed and slows the motor
																			// down as it gets nearer to its stopping
																			// point

				if (liftspeed >= 1.0) {
					liftspeed = 1.0;// the motor cannot run faster than 1.0, so if a faster run is requested, this
									// will constrain the value
				}
				if (liftspeed <= -0.8) {
					liftspeed = 0.8;
				}

				m_liftMotor.set(liftspeed);// run the motor

			}
			if (lifttopMax) {
				m_liftMotor.set(0);
			}

			if (level2Req && m_encoder.getPosition() <= 500 && !lifttopMax) {// same as above

				double liftspeed = (m_encoder.getPosition() - 500) / -20;

				if (liftspeed >= 1.0) {
					liftspeed = 1.0;
				}

				m_liftMotor.set(liftspeed);

			}

			if ((level2Req && m_encoder.getPosition() >= 500) || lifttopMax) {
				m_liftMotor.set(0);
			}

			if (!levelReq && !level2Req && !downRequest) {// if no levels requested stop the motor
				m_liftMotor.set(0);
			}

			if (downRequest && m_encoder.getPosition() >= 40) {// same as above but for moving down

				double liftspeed = (m_encoder.getPosition()) / -20;

				if (liftspeed <= -0.6) {
					liftspeed = -0.6;
				}

				m_liftMotor.set(liftspeed);

			}
			if (downRequest && m_encoder.getPosition() <= 40) {
				m_liftMotor.set(0);
			}
		}

		if (!ballIn && !autoLockout && !techLock && m_encoder.getPosition() <=510 && !lifttopMax) {// same as the whole big chunk above but with slightly different
													// values

			if (levelReq && m_encoder.getPosition() <= 100 && !lifttopMax) {

				double liftspeed = (m_encoder.getPosition() - 100) / -10;

				if (liftspeed >= 1.0) {
					liftspeed = 1.0;
				}

				m_liftMotor.set(liftspeed);

			}
			if ((levelReq && m_encoder.getPosition() >= 100) || lifttopMax) {

				m_liftMotor.set(0);
			}

			if (level2Req && m_encoder.getPosition() <= 500 && !lifttopMax) {

				double liftspeed = (m_encoder.getPosition() - 500) / -10;

				if (liftspeed >= 1.0) {
					liftspeed = 1.0;
				}

				m_liftMotor.set(liftspeed);

			}

			if ((level2Req && m_encoder.getPosition() >= 500) || lifttopMax) {
				m_liftMotor.set(0);
			}

			if (!levelReq && !level2Req && !downRequest) {
				m_liftMotor.set(0);
			}

			if (downRequest && m_encoder.getPosition() >= 40) {

				double liftspeed = (m_encoder.getPosition()) / -10;

				if (liftspeed <= -0.6) {
					liftspeed = -0.6;
				}

				m_liftMotor.set(liftspeed);

			}
			if (downRequest && m_encoder.getPosition() <= 40) {
				m_liftMotor.set(0);
			}
		}
		/***************************************************************************************************/

		if (techLock && level2d && driverstick.getPOV() != -1 && techstick.getPOV() != -1) {

			m_ballIn.set(ControlMode.PercentOutput, -1.0);
		}

		if (techLock && techstick.getRawButton(6) && driverstick.getPOV() != -1 && techstick.getPOV() != -1) {
			m_eject.set(ControlMode.PercentOutput, -0.8);
		}

		if (!techLock && !level2d && !techstick.getRawButton(6) && driverstick.getPOV() == -1
				&& techstick.getPOV() == -1) {

			m_ballIn.set(ControlMode.PercentOutput, 0);
			m_eject.set(ControlMode.PercentOutput, 0);

		}

		/***************************************************************************************************/
		if (m_encoder.getPosition() >= 250) {

			elevatorStatus = "reached level 1";// if the lift reaches position 250, send this to smartdashboard
		}

		if (m_encoder.getPosition() >= 500) {

			elevatorStatus = "reached level 2";
		}

		if (levelReq) {
			requestPosition = "level1 Requested";// these three if statements send what level was requested to
													// smartdashboard
		}
		if (level2Req) {
			requestPosition = "level2 Requested";
		}

		if (downRequest) {
			requestPosition = "lowering Requested";
		}

		SmartDashboard.putNumber("elevator height", m_encoder.getPosition());// put the stuff onto smartdashboard
		SmartDashboard.putString("elevator status", elevatorStatus);
		SmartDashboard.putString("Ball status", bucket);
		SmartDashboard.putString("level Requested", requestPosition);
		SmartDashboard.putString("target detect", targetDetect);
		SmartDashboard.putNumber("sensor distance", range);

	}

	@Override
	public void testPeriodic() {
	}
}
