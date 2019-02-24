package ca.mcgill.ecse211.lab5;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

import static ca.mcgill.ecse211.lab5.Lab5.SENSOR_MOTOR;

import java.util.Arrays;

import ca.mcgill.ecse211.odometer.*;



public class ColorDetection extends Thread {
	
	 private static final double CAN_EXISTS = 10;
	 private float[] usData;
	 private SampleProvider usDistance;
	 private float[] lightData;
	 private SampleProvider lightColor;
	 public int colorIndex;
	 
	 private int filterSum;
	 CanCalibrator calibrator;
	 
	// the display LCD screen object
	 private TextLCD lcd;
	
	public ColorDetection (SampleProvider usDistance, float[] usData, SampleProvider lightColor, float[] lightData, TextLCD lcd){
		this.usDistance = usDistance;
		this.usData = usData;
		this.lightColor = lightColor;
        this.lightData = lightData;
        this.lcd = lcd;
        calibrator = new CanCalibrator(lightColor, lightData);
	}
	public void run() {
		
		while(true){
			
			if (isCan()){
				lcd.drawString("Object Detected!!!", 0, 0);
				Sound.beep();
				
				colorIndex = rotateSensorDetect();
				if (colorIndex == 0){
					lcd.drawString("Color detected: Red", 0, 1);
				}
				else if (colorIndex == 1){
					lcd.drawString("Color detected: Green", 0, 1);
				}
				else if (colorIndex == 2){
					lcd.drawString("Color detected: Blue", 0, 1);
				}
				else if (colorIndex == 3){
					lcd.drawString("Color detected: Yellow", 0, 1);
				}
				try {
					Thread.sleep(6000);
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
				
				
				//Wait 6 second for check reading
			
				lcd.clear();
				
			}
			
		} 
	}
	
	
	private int rotateSensorDetect(){
		int[] colorResult = new int[7];
		for (int i = 0; i < 7; i++){
			colorResult[i] = calibrator.Calibrate();
			Lab5.SENSOR_MOTOR.setAcceleration(300);
			Lab5.SENSOR_MOTOR.setSpeed(300);
			Lab5.SENSOR_MOTOR.rotate(30, false);
		}
		SENSOR_MOTOR.rotateTo(0, false);
		Arrays.sort(colorResult);
		int prev = colorResult[0];
		int count = 1;
		for(int i = 1; i<colorResult.length; i++){
			if (colorResult[i] == prev && colorResult[i] != -1){
				count ++;
				if(count > colorResult.length / 2 -1 ) {return colorResult[i];}
			}else{
				count = 1;
				prev = colorResult[i];
			}
		}
		return -1;
	}
	private boolean isCan() {
	    double distance = medianFilter();
	    if (distance < CAN_EXISTS) {
	    	try {
				Thread.sleep(1000);
			} catch (InterruptedException e) {
				// TODO Auto-generated catch block
				e.printStackTrace();
			}
	    	
	    	 if (distance < CAN_EXISTS){
	    		 return true;
	    	 }
	    }
	    return false;
	  }

	/**
	 * This is a median filter. The filter takes 5 consecutive readings from the ultrasonic sensor,
	 * amplifies them to increase sensor sensitivity, sorts them, and picks the median to minimize the
	 * influence of false negatives and false positives in sensor readings, if any. The sensor is very
	 * likely to report false negatives.
	 * 
	 * @return the median of the five readings, sorted from small to large
	 */
	private double medianFilter() {
		double[] arr = new double[5]; // store readings
		for (int i = 0; i < 5; i++) { // take 5 readingss
			usDistance.fetchSample(usData, 0); // store reading in buffer
			arr[i] = usData[0] * 100.0; // signal amplification
		}
		Arrays.sort(arr); // sort readingss
		return arr[2]; // take median value
	}
	
	 private double meanFilter() {
		 filterSum = 0;
		 // take 5 readings
		 for (int i = 0; i < 5; i++) {

		 // acquire sample data and read into array with no offset
		 lightColor.fetchSample(lightData, 0);

		  // amplify signal for increased sensitivity
		  filterSum += lightData[0] * 100;

		 }

		 // return an amplified average
		 return filterSum / 5.0;

	 }

}
