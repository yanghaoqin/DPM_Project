package ca.mcgill.ecse211.lab5;

import static ca.mcgill.ecse211.lab5.Lab5.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.BLUE_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.YELLOW_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.GREEN_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.RED_COLOR;
import static ca.mcgill.ecse211.lab5.Lab5.SENSOR_MOTOR;
import java.util.Arrays;
import ca.mcgill.ecse211.odometer.*;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import lejos.robotics.navigation.Navigator;
import ca.mcgill.ecse211.lab5.Navigation;

/**
 * 
 * @author Yinuo Wang
 * @author Raymond Yang
 * @author Tudor Gurau
 *
 */
public class Search extends Thread {


  private final int SCAN_TIME = 100;
  // {R, G, B} values based on in lab measurements and multiplied by 100
  public static final int LLx = 2; // lower left x coordinate of searching area, modify during demo
  public static final int LLy = 2; // lower left y coordinate of searching area, modify during demo
  public static final int URx = 4; // lower left x coordinate of searching area, modify during demo
  public static final int URy = 4 ; // lower left y coordinate of searching area, modify during demo

  private static final int RED_INDEX = 4;
  private static final int GREEN_INDEX = 2;
  private static final int BLUE_INDEX = 1;
  private static final int YELLOW_INDEX = 3;

  private static final int TR = 4; // colour of target can: must be changed during demo
  public static final double CAN_EXISTS = 15; // distance to show there is a can at that
                                              // intersection, TODO: tweak in lab

  private static double[] threshold; // colour threshold to identify correct can
  private Odometer odometer; // odometer
  private float[] usData;
  private SampleProvider usDistance;
  private float[] lightData;
  private SampleProvider lightColor;
  private float[] csData;
  private SampleProvider csColor;
  private int filterSum;
  private double csfilterSum;
  private int std;
  private boolean isCan;
  private Navigation nav;
  private double[] colorData;
  private CanCalibrator calibrator;

  public static double distDisplay;

  public Search(Odometer odometer, SampleProvider usDistance, float[] usData,
      SampleProvider lightColor, float[] lightData, SampleProvider csColor, float[] csData) {
    this.odometer = odometer;
    this.usData = usData;
    this.usDistance = usDistance;
    this.lightData = lightData;
    this.lightColor = lightColor;
    this.csData = csData;
    this.csColor = csColor;
    this.isCan = false;
    this.colorData = new double[4];
    nav = new Navigation(odometer, usDistance, usData);

    // determine which color from TR given
    switch (TR) {
      case 1: // TR = 1 --> blue can
        threshold = BLUE_COLOR;
        break;
      case 2: // TR = 2 --> green can
        threshold = GREEN_COLOR;
        break;
      case 3: // TR = 3 --> yellow can
        threshold = YELLOW_COLOR;
        break;
      case 4: // TR = 4 --> red can
        threshold = RED_COLOR;
    }
  }

  public void run() {
    // scan grid
    boolean canFound = false;
    
    for (int y = LLy; y <= URy; y++) {
    	int x = 0;
      
    
      canFound = false;
      
      if ((y - LLy) % 2 == 0) {
        for ( x = LLx; x <= URx; x++) {
          canFound = false;
          while (canFound == false) {
            isCan = nav.travelTo(x, y);
            if (isCan) {
              canFound = canFound();
            } else {
              canFound = true;
            }
          }
        }
        lineLocalRight(x, y); 
      } else {
        if ((y - LLy) % 2 == 1) {
          for ( x = URx; x >= LLx; x--) {
            canFound = false;
            while (canFound == false) {
              isCan = nav.travelTo(x, y);
              if (isCan) {
                canFound = canFound();
              } else {
                canFound = true;
              }
            }
          }
        }
      }

  
  	
    }
  }

  // /**
  // *
  // * @return
  // */
  // private boolean isCan() {
  // double distance = medianFilter();
  // if (distance < CAN_EXISTS) {
  // Sound.beepSequenceUp();
  //
  // LEFT_MOTOR.rotate(convertDistance(WHEEL_RAD, 12), true);
  // RIGHT_MOTOR.rotate(convertDistance(WHEEL_RAD, 12), false);
  // return true;
  // }
  // return false;
  // }

  private boolean canFound() {
    // double[] red_array = new double[100]; // array for reds
    // double[] green_array = new double[100]; // array for greens
    // double[] blue_array = new double[100]; // array for blues

    calibrator = new CanCalibrator(lightColor, lightData);
    // double starting_angle = odometer.getXYT()[2];

    LEFT_MOTOR.rotate(convertDistance(Lab5.WHEEL_RAD, 10), true);
    RIGHT_MOTOR.rotate(convertDistance(Lab5.WHEEL_RAD, 10), false);

    // check if color reading is correct
    int color = rotateSensorDetect();

    LEFT_MOTOR.rotate(-convertDistance(Lab5.WHEEL_RAD, 9), true);
    RIGHT_MOTOR.rotate(-convertDistance(Lab5.WHEEL_RAD, 9), false);

    if (color == colorconvert(TR)) {
      // navigate to the end position
      Sound.beep();
      hitIt(true);
      nav.travelTo(URx, URy);
      System.exit(0);
      return true;
    } else {
      // navigate to the next position
      Sound.twoBeeps();
      hitIt();
      return false;
    }
  }

  private int colorconvert(int tr) {
    switch (tr) {
      case 1:
        return 2;
      case 2:
        return 1;
      case 3:
        return 3;
      case 4:
        return 0;
    }
    return -1;
  }

  private int rotateSensorDetect() {
    int[] colorResult = new int[7];
    for (int i = 0; i < 7; i++) {
      colorResult[i] = calibrator.Calibrate();
      Lab5.SENSOR_MOTOR.setAcceleration(300);
      Lab5.SENSOR_MOTOR.setSpeed(300);
      Lab5.SENSOR_MOTOR.rotate(30, false);
    }
    SENSOR_MOTOR.rotateTo(0, false);
    Arrays.sort(colorResult);
    int prev = colorResult[0];
    int count = 1;
    for (int i = 1; i < colorResult.length; i++) {
      if (colorResult[i] == prev && colorResult[i] != -1) {
        count++;
        if (count > colorResult.length / 2 - 1) {
          return colorResult[i];
        }
      } else {
        count = 1;
        prev = colorResult[i];
      }
    }
    return -1;
  }

  /**
   * oh yeah
   */
  private void hitIt() {
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);
    
//    SENSOR_MOTOR.setSpeed(SENSOR_MOTOR.getMaxSpeed());
//    SENSOR_MOTOR.setAcceleration(5000);
    
    SENSOR_MOTOR.rotate(220, false);
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);
    SENSOR_MOTOR.rotateTo(0, false);
  }
  
  private void hitIt(boolean flag) {
    SENSOR_MOTOR.setSpeed(SENSOR_MOTOR.getMaxSpeed());
    SENSOR_MOTOR.setAcceleration(5000);
    
    SENSOR_MOTOR.rotate(220, false);
    SENSOR_MOTOR.setSpeed(100);
    SENSOR_MOTOR.setAcceleration(500);
    SENSOR_MOTOR.rotateTo(0, false);
  }

  /**
   * initial readings taken (100 times) and the average is used to distinguish between the wooden
   * board and the black line. Each reading is amplified to enhance the sensitivity of the sensor
   * 
   * @return
   */
  private double initialReading(int color_id) {
    for (int i = 0; i < 100; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, color_id);
      // amplifies and sums the sample data
      std += lightData[color_id] * 100.0;
    }
    // take average of the standard
    return std /= 100.0;
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

  /**
   * This is a static method allows the conversion of a distance to the total rotation of each wheel
   * need to cover that distance.
   * 
   * (Distance / Wheel Circumference) = Number of wheel rotations. Number of rotations * 360.0
   * degrees = Total number of degrees needed to turn.
   * 
   * @param radius - Radius of the wheel
   * @param distance - Distance of path
   * @return an integer indicating the total rotation angle for wheel to cover the distance
   */
  private static int convertDistance(double radius, double distance) {
    return (int) ((180.0 * distance) / (Math.PI * radius));
  }

  /**
   * This is a static method that converts the angle needed to turn at a corner to the equivalent
   * total rotation. This method first converts the degrees of rotation, radius of wheels, and width
   * of robot to distance needed to cover by the wheel, then the method calls another static method
   * in process to convert distance to the number of degrees of rotation.
   * 
   * @param radius - the radius of the wheels
   * @param width - the track of the robot
   * @param angle - the angle for the turn
   * @return an int indicating the total rotation sufficient for wheel to cover turn angle
   */
  private static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
  /**
	 * This method let the robot to approach the origin
	 * and then backup for a longer speed to ensure that all four lines are included in the rotating radius 
	 */
	public void approachOrigin() {
		// Turn 45 degree towards the origin
		 RIGHT_MOTOR.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 45), true);
	     LEFT_MOTOR.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, 45), false);
		

		LEFT_MOTOR.setSpeed(150);
		RIGHT_MOTOR.setSpeed(150);
		
		csfilterSum = fetchSample();
		// move forward past the origin until light sensor sees the line
		while (csfilterSum > 0.38) {
			csfilterSum = fetchSample();		
			LEFT_MOTOR.forward();
			RIGHT_MOTOR.forward();

		}
		
		LEFT_MOTOR.stop(true);
		RIGHT_MOTOR.stop();
		Sound.beep();

		// Move backwards so our origin is close to origin
		// Always back more (-6) than it forwarded so the car center is ways at negative X and Y
		// Consistent with later calculation
		LEFT_MOTOR.rotate(convertDistance(Lab5.WHEEL_RAD, -13 - 6), true);
		RIGHT_MOTOR.rotate(convertDistance(Lab5.WHEEL_RAD, -13 - 6), false);

	}
	private void lineLocalRight(int x, int y){
		approachOrigin();

		// Scan all four lines and record our angle
  	int index = 0;
	double lineData[] = new double[4];
	
		while (index < 4) {
			LEFT_MOTOR.backward();
			RIGHT_MOTOR.forward();
			csfilterSum = fetchSample();
			if (csfilterSum < 0.38 ) {
				lineData[index] = odometer.getXYT()[2];  // Get theta value and store it inside the array by rising index
				index++;
				Sound.beep();
			}
		}

		LEFT_MOTOR.stop(true);
		RIGHT_MOTOR.stop();

		double d_X, d_Y, Theta_X, Theta_Y;

		// calculate our location from 0 using the calculated angles
		Theta_Y = lineData[2] - lineData[0];
		Theta_X = lineData[3] - lineData[1];
		
		// calculate the difference of angle on the odometer and it in reality
		double d_theta = 90 - (lineData[2]-180)+ Theta_Y/2.0;  

		d_X = -Math.abs(13 * Math.cos(Math.toRadians(Theta_Y / 2.0)));
		d_Y = -Math.abs(13 * Math.cos(Math.toRadians(Theta_X / 2.0)));
		
	
		// travel to origin to correct position
		odometer.setXYT(d_X, d_Y, odometer.getXYT()[2]);
		nav.travelTo(0.0, 0.0);

		// if we are not facing 0.0 then turn ourselves so that we are	
		if (odometer.getXYT()[2] <= 350 && odometer.getXYT()[2] >= 10.0) {
			Sound.beep();
			LEFT_MOTOR.setSpeed(100 / 2);
			RIGHT_MOTOR.setSpeed(100 / 2);

			// Current theta value plus the calculated error is the actual heading
			// Then turn negatively (to the right) to offset the angle to zero
			LEFT_MOTOR.rotate(-convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, odometer.getXYT()[2] + d_theta), true);
			RIGHT_MOTOR.rotate(convertAngle(Lab5.WHEEL_RAD, Lab5.TRACK, odometer.getXYT()[2] + d_theta), false);
		}
		LEFT_MOTOR.stop(true);
		RIGHT_MOTOR.stop();

		//After localizing, initializing the odometer
		odometer.setX((x-1)*Lab5.TILE);	
				
		odometer.setY(y*Lab5.TILE);
  
	}
		private double fetchSample(){
		csfilterSum = 0;
	    // take 5 readings
	    for (int i = 0; i < 5; i++) {
	      // acquire sample data and read into array with no offset
	      csColor.fetchSample(csData, 0);
	      // amplify signal for increased sensitivity
	      csfilterSum += csData[0] ;
	    }
	    // return an amplified average
	    return csfilterSum / 5.0;

	}
}
