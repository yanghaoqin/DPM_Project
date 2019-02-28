package ca.mcgill.ecse211.localization;

import lejos.hardware.Sound;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import ca.mcgill.ecse211.odometer.*;
//import ca.mcgill.ecse211.localization.*;
import ca.mcgill.ecse211.navigation.*;
import ca.mcgill.ecse211.lab4.*;

/**
 * 
 * @author Tudor Gurau
 * @author Antoine Wang
 *
 */

public class LightLocalizer {

	// vehicle constants
	public static int ROTATION_SPEED = 100;
	private double SENSOR_LENGTH = 14.0;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	public Navigation navigation;
	// Instantiate the EV3 Color Sensor
	private static final EV3ColorSensor lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S3"));
	private float sample;

	private SensorMode idColour;

	double[] lineData;

	public LightLocalizer(Odometer odometer, EV3LargeRegulatedMotor leftMotor, EV3LargeRegulatedMotor rightMotor) {

		this.odometer = odometer;
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;

		idColour = lightSensor.getRedMode(); // set the sensor light to red
		lineData = new double[4];
		navigation = new Navigation(odometer, leftMotor, rightMotor);
	}

	/**
	 * This method localizes the robot using the light sensor to precisely move to
	 * the right location
	 */
	public void localize() {

		int index = 0;
		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		// ensure that we are close to origin before rotating
		moveToOrigin();

		// Scan all four lines and record our angle
		while (index < 4) {
			leftMotor.backward();
			rightMotor.forward();
			sample = fetchSample();
			if (sample < 0.38) {
				lineData[index] = odometer.getXYT()[2];
				index++;
				Sound.beep();
			}
		}

		leftMotor.stop(true);
		rightMotor.stop();

		double deltax, deltay, thetax, thetay;

		// calculate our location from 0 using the calculated angles
		thetay = lineData[2] - lineData[0];
		thetax = lineData[3] - lineData[1];
		
		double d_theta = 90 - (lineData[2]-180)+thetay/2;

		deltax = -Math.abs(SENSOR_LENGTH * Math.cos(Math.toRadians(thetay / 2)));
		deltay = -Math.abs(SENSOR_LENGTH * Math.cos(Math.toRadians(thetax / 2)));
		
		//pause 1s
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

		// travel to origin to correct position
		odometer.setXYT(deltax, deltay, odometer.getXYT()[2]);
		navigation.travelTo(0.0, 0.0);

		// if we are not facing 0.0 then turn ourselves so that we are	
		if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
			Sound.beep();
			leftMotor.setSpeed(ROTATION_SPEED / 2);
			rightMotor.setSpeed(ROTATION_SPEED / 2);

			leftMotor.rotate(-convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, odometer.getXYT()[2] + d_theta), true);
			rightMotor.rotate(convertAngle(DPM_Lab4.WHEEL_RAD, DPM_Lab4.TRACK, odometer.getXYT()[2] + d_theta), false);
		}
		leftMotor.stop(true);
		rightMotor.stop();

	}

	/**
	 * This method moves the robot towards the origin
	 */
	public void moveToOrigin() {

		navigation.turnTo(Math.PI / 4);

		leftMotor.setSpeed(ROTATION_SPEED);
		rightMotor.setSpeed(ROTATION_SPEED);

		// get sample
		sample = fetchSample();

		// move forward past the origin until light sensor sees the line
		while (sample > 0.38) {
			sample = fetchSample();
			leftMotor.forward();
			rightMotor.forward();

		}
		leftMotor.stop(true);
		rightMotor.stop();
		Sound.beep();

		// Move backwards so our origin is close to origin
		leftMotor.rotate(convertDistance(DPM_Lab4.WHEEL_RAD, -SENSOR_LENGTH-6), true);
		rightMotor.rotate(convertDistance(DPM_Lab4.WHEEL_RAD, -SENSOR_LENGTH-6), false);

	}

	/**
	 * This method allows the conversion of a distance to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius radius of wheel
	 * @param distance distance traveled
	 * @return degree that wheel should rotate
	 */
	private static int convertDistance(double radius, double distance) {
		return (int) ((180.0 * distance) / (Math.PI * radius));
	}

	/**
	 * This method allows the conversion of a angle to the total rotation of each
	 * wheel need to cover that distance.
	 * 
	 * @param radius wheel radius
	 * @param distance distance traveled
	 * @param angle angle in degree
	 * @return degree that each wheel needs to turn
	 */
	private static int convertAngle(double radius, double width, double angle) {
		return convertDistance(radius, Math.PI * width * angle / 360.0);
	}

	/**
	 * This method gets the color value of the light sensor
	 * 
	 */
	private float fetchSample() {
		float[] colorValue = new float[idColour.sampleSize()];
		idColour.fetchSample(colorValue, 0);
		return colorValue[0];
	}

}