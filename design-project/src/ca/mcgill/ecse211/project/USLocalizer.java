package ca.mcgill.ecse211.project;

import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.robotics.SampleProvider;
//import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.odometer.*;
import ca.mcgill.ecse211.project.*;
/**
 * This class enables the robot to localize itself in a corner based on ultrasonic sensor readings.
 * 
 * @author Tudor Gurau
 * @author Antoine Wang
 *
 */
public class USLocalizer {

	// vehicle constants
	private static final int ROTATION_SPEED = 100;
	private double deltaTheta;

	private Odometer odometer;
	private float[] usData;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private int buttonChoice;
	private SampleProvider usDistance;

	// Create a navigation
	public Navigation navigation;

	private static final double distance = 18.00;
	private static final int tolerance = 2;
	private static final double distance_rising = 30.00;
	private static final int tolerance_rising = 2;

	/**
	 * This is the constructor for the USLocalizer.
	 * 
	 * @param odo the Odometer of the robot
	 * @param leftMotor the Left Motor of the robot
	 * @param rightMotor the Right Motor of the robot
	 * @param localizationType the Localization type (rising or falling edge)
	 * @param usDistance the Sample Provider of the ultrasonic sensor
	 */
	public USLocalizer(Odometer odo, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor,
			int localizationType, SampleProvider usDistance) {
		this.odometer = odo;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.buttonChoice = localizationType;
		this.usDistance = usDistance;
		this.usData = new float[usDistance.sampleSize()];
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);
	}

	/**
	 * This method determines which localization type is demanded
	 * 
	 */
	public void localize() {
		if (buttonChoice == Button.ID_RIGHT) {
			localizeRisingEdge();
		} else if (buttonChoice == Button.ID_LEFT) {
			localizeFallingEdge();
		}
	}

	/**
	 * This method localizes the robot's position using the rising edge
	 * 
	 */
	public void localizeRisingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to the wall
		while (fetchUS() > distance_rising) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate until it sees the open space
		while (fetchUS() < distance_rising + tolerance_rising) {
			leftMotor.backward();
			rightMotor.forward();
		}
		Sound.buzz();
		// record angle
		angleA = odometer.getXYT()[2];

		// rotate the other way all the way until it sees the wall
		while (fetchUS() > distance_rising) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate until it sees open space
		while (fetchUS() < distance_rising + tolerance_rising) {
			leftMotor.forward();
			rightMotor.backward();
		}
		Sound.buzz();
		angleB = odometer.getXYT()[2];

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2 + 180;
		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2 + 180;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 using turning angle and we account for small
		// error
		leftMotor.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, turningAngle), true);
		rightMotor.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, turningAngle), false);

		// set theta = 0.0
		odometer.setXYT(0.0, 0.0, 0.0);
		
	}

	/**
	 * This method localizes the robot's position using the falling edge
	 * 
	 */
	public void localizeFallingEdge() {

		double angleA, angleB, turningAngle;

		// Rotate to open space
		while (fetchUS() < distance + tolerance) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// Rotate to the first wall
		while (fetchUS() > distance) {
			leftMotor.backward();
			rightMotor.forward();
		}
		// record angle
		angleA = odometer.getXYT()[2];
		Sound.buzz();

		// rotate out of the wall range
		while (fetchUS() < distance + tolerance) {
			leftMotor.forward();
			rightMotor.backward();
		}

		// rotate to the second wall
		while (fetchUS() > distance) {
			leftMotor.forward();
			rightMotor.backward();
		}
		angleB = odometer.getXYT()[2];
		Sound.buzz();

		leftMotor.stop(true);
		rightMotor.stop();

		// calculate angle of rotation
		if (angleA < angleB) {
			deltaTheta = 45 - (angleA + angleB) / 2;

		} else if (angleA > angleB) {
			deltaTheta = 225 - (angleA + angleB) / 2;
		}

		turningAngle = deltaTheta + odometer.getXYT()[2];

		// rotate robot to the theta = 0.0 and we account for small error
		leftMotor.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, turningAngle ), true);
		rightMotor.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, turningAngle ), false);

		// set odometer to theta = 0
		odometer.setXYT(0.0, 0.0, 0.0);

	}

	/**
	 * This method gets the distance from the ultrasonic sensor
	 * 
	 * @return distance detected by US sensor
	 */
	private int fetchUS() {
		usDistance.fetchSample(usData, 0);
		return (int) (usData[0] * 100);
	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel needed to cover that distance.
	 * 
	 * @param radius wheel radius
	 * @param distance distance traveled
	 * @return degree that wheel should turn
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of an angle to the total rotation of each
	 * wheel needed to cover that distance.
	 * 
	 * @param radius wheel radius
	 * @param distance distance traveled
	 * @param angle angle in degree
	 * @return degree that each wheel needs to turn
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

}