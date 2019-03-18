package ca.mcgill.ecse211.project;

import lejos.hardware.Sound;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorMode;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;
import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import ca.mcgill.ecse211.odometer.*;

public class DoubleLightLocalization {

  private Odometer odometer;
  private SampleProvider LeftidColour;
  private SampleProvider RightidColour;
  private float[] LeftcolorValue;
  private float[] RightcolorValue;
  
  public static final double SENSOR_TOWHEEL = 12.6; 
  private static final double BACK_DIST = 15.6; 
  /**
   * 
   * @param odometer Instance of the Odometer class 
   * @param navigator Instance of the Navigation class
   * @param LeftcsSensor Left side color sensor
   * @param RightcsSensor Right side color sensor
   */
  public DoubleLightLocalization(Odometer odometer, SampleProvider LeftcsSensor,
      SampleProvider RightcsSensor, float[] LeftcolorValue, float[] RightcolorValue) {

    this.odometer = odometer;
    this.LeftidColour = LeftcsSensor;
    this.RightidColour = RightcsSensor;
    this.LeftcolorValue = LeftcolorValue;
    this.RightcolorValue = RightcolorValue;
    
    // set the sensor light to red
  
  }

  /**
   * Localize the machine using 2 light sensors:
   * 1. With the robot heading roughly in positive y direction, drive forward until both sensor detected x axis
   * 2. Reset odometer value (Y coord) and the heading according to X axis
   * 3. Drive back 20cm to ensure the machine is in the 3rd quadrant relative to the origin
   * 4. turn 90 degree to the right. With the robot heading roughly in positive x direction, drive forward until both sensor detected y axis
   * 5. Reset odometer value (X coord) and the heading according to Y axis
   * 6. Drive back 20cm to ensure the machine is in the 3rd quadrant relative to the origin, then navigate to the original point.
   * 7. At the end of the method, the vehicle should be located at (0,0) with a heading of 0 degree
   */

  public void DoubleLocalizer() {

	// Drive to X axis and initialize Y
    travelToLine();
    odometer.setY(SENSOR_TOWHEEL + project.TILE);
    odometer.setTheta(0);
    
    //Drive back 20cm and turn 90 degree facing Y axis
    RIGHT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, BACK_DIST), true);
    LEFT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, BACK_DIST), false);
    
    reorientRobot(Math.PI/2);
    // Drive to y axis and initialize x
    travelToLine();
    odometer.setX(SENSOR_TOWHEEL + project.TILE);
    odometer.setTheta(90);
    
    // Drive back for 20cm and then localize at the origin
    RIGHT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, BACK_DIST), true);
    LEFT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, BACK_DIST), false);
    travelToOrigin();
    
    odometer.setXYT(project.TILE, project.TILE, odometer.getXYT()[2]);
    odometer.Theta = 0;
    
  }

  /**
   * Travel to the front until the 2 light sensors poll a black line
   */
  private void travelToLine() {

    project.LEFT_MOTOR.setAcceleration(6000);
    project.RIGHT_MOTOR.setAcceleration(6000);
    
    project.LEFT_MOTOR.setSpeed(200);
    project.RIGHT_MOTOR.setSpeed(200);


    boolean counter_left = true;
    boolean counter_right = true;
    while (counter_left || counter_right) {
      if(counter_left) {
        project.LEFT_MOTOR.forward();
        
      }
      
      if(counter_right) {
        project.RIGHT_MOTOR.forward();
      
      }
       
      if(fetchSampleLeft() < 0.38 && counter_left) {
        project.LEFT_MOTOR.stop(true);
        project.RIGHT_MOTOR.stop();
        Sound.beep();
        counter_left = false;
        try {
          Thread.sleep(200);
        }
        catch(Exception e) {
          
        }
        //project.RIGHT_MOTOR.rotate(-90);
        
        project.RIGHT_MOTOR.forward();
        if(fetchSampleLeft() < 0.38 && !counter_right) {
          project.LEFT_MOTOR.stop(true);
          project.RIGHT_MOTOR.stop();
          break;      
        }
      }
       
      if(fetchSampleRight() < 0.38 && counter_right) {
        project.RIGHT_MOTOR.stop(true);
        project.LEFT_MOTOR.stop();
        Sound.buzz();
        counter_right = false;
        try {
          Thread.sleep(200);
        }
        catch(Exception e) {
          
        }
        //project.LEFT_MOTOR.rotate(-90);
        
        project.LEFT_MOTOR.forward();
        if(fetchSampleRight() < 0.38 && !counter_left) {
          project.LEFT_MOTOR.stop(true);
          project.RIGHT_MOTOR.stop();
          break;
        }
      }
    }
      
  }

  /**
   * polls the left light sensor for the ground
   * @return current sample of the color
   */
  private double fetchSampleLeft() {
    double filterSum = 0;
    for(int i = 0; i < 10; i++) {
      LeftidColour.fetchSample(LeftcolorValue, 0);
      filterSum += LeftcolorValue[0];
    }
    return filterSum / 10;
  }

  /**
   * polls the right light sensor for the ground
   * @return current sample of the color
   */
  private double fetchSampleRight() {
    double filterSum = 0;
    for(int i = 0; i < 10; i++) {
      RightidColour.fetchSample(RightcolorValue, 0);
      filterSum += RightcolorValue[0];
    }
    return filterSum / 10;
  }
  
  /**
   * This method turns the robot by the magnitude indicated by the input parameter
   * The input angle will always be brought down to the minimum angle needed to turn and
   * it can adjust itself in both cw and ccw directions. 
   * @param theta RADIAN value of the angle that the robot will adjust its orientation
   * 		For adjusting clockwise the parameter should be positive, for ccw adjustment please input a negative radian
   */
  public static void reorientRobot(double theta) {

    // ensures minimum angle for turning
    if (theta > Math.PI) {
        theta -= 2 * Math.PI;
    } else if (theta < -Math.PI) {
        theta += 2 * Math.PI;
    }

    // set Speed
    project.LEFT_MOTOR.setSpeed(200);
    project.RIGHT_MOTOR.setSpeed(200);

    // rotate motors at set speed

    // if angle is negative, turn to the left
    if (theta < 0) {
      project.LEFT_MOTOR.rotate(-Navigation.convertAngle(project.WHEEL_RAD, project.TRACK, -(theta * 180) / Math.PI), true);
      project.RIGHT_MOTOR.rotate(Navigation.convertAngle(project.WHEEL_RAD, project.TRACK, -(theta * 180) / Math.PI), false);

    } else {
        // angle is positive, turn to the right
      project.LEFT_MOTOR.rotate(Navigation.convertAngle(project.WHEEL_RAD, project.TRACK, (theta * 180) / Math.PI), true);
      project.RIGHT_MOTOR.rotate(-Navigation.convertAngle(project.WHEEL_RAD, project.TRACK, (theta * 180) / Math.PI), false);
    }
}
  
  /**
   * This method takes in the robot's current location and navigate the robot to the (0,0) point
   * Then it adjust the robot's orientation into zero degree (facing positive Y axis)
   */
  public void travelToOrigin() {
    double currx;
    double curry;
    double currTheta;
    double deltax;
    double deltay;
    
    //Get current location from odometer
    currx = odometer.getXYT()[0];
    curry = odometer.getXYT()[1];

    deltax = project.TILE - currx;
    deltay = project.TILE - curry;

    // Calculate the angle to turn around
    currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
    double mTheta = Math.atan2(deltax, deltay) - currTheta;

    //Calculate the distance from original point
    double hypot = Math.hypot(deltax, deltay);

    // Turn to the correct angle towards the endpoint
    reorientRobot(mTheta);

    //Setting the speed and proceed to the origin point
    project.LEFT_MOTOR.setSpeed(180);
    project.RIGHT_MOTOR.setSpeed(180);
    project.LEFT_MOTOR.rotate(Navigation.convertDistance(project.WHEEL_RAD, hypot), true);
    project.RIGHT_MOTOR.rotate(Navigation.convertDistance(project.WHEEL_RAD, hypot), false);
    
    //Adjusting itself at 0 degree heading by turning in the opposite direction to offset the current odometer reading
    LEFT_MOTOR.rotate(-Navigation.convertAngle(project.WHEEL_RAD, project.TRACK, odometer.getXYT()[2]), true);
    RIGHT_MOTOR.rotate(Navigation.convertAngle(project.WHEEL_RAD, project.TRACK, odometer.getXYT()[2]), false);

    // stop vehicle
    project.LEFT_MOTOR.stop(true);
    project.RIGHT_MOTOR.stop(false);
    
    
  }
  

  
 
}
