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
  private Navigation navigator;
  private SampleProvider LeftidColour;
  private SampleProvider RightidColour;
  private float[] LeftcolorValue;
  private float[] RightcolorValue;
  
  double[] lineData;

  /**
   * 
   * @param odometer Instance of the Odometer class 
   * @param navigator Instance of the Navigation class
   * @param LeftcsSensor Left side color sensor
   * @param RightcsSensor Right side color sensor
   */
  public DoubleLightLocalization(Odometer odometer, Navigation navigator, SampleProvider LeftcsSensor,
      SampleProvider RightcsSensor, float[] LeftcolorValue, float[] RightcolorValue) {

    this.odometer = odometer;
    this.navigator = navigator;
    this.LeftidColour = LeftcsSensor;
    this.RightidColour = RightcsSensor;
    this.LeftcolorValue = LeftcolorValue;
    this.RightcolorValue = RightcolorValue;
    
    // set the sensor light to red
    lineData = new double[4];
    // navigation = new Navigation(odometer);
  }

  /**
   * Localize the machine using 2 light sensors: travel to the black line for x,
   * turn 90, travel to a black line for y, turn -90 fall back for 20 cm, advance until we
   * see lines, back up by 12.6 cm (length between sensor and wheel).
   * reset odometer
   */

  public void DoubleLocalizer() {

    travelToLine();
    odometer.setY(12.6);
    odometer.setTheta(0);
    RIGHT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, 20), true);
    LEFT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, 20), false);
    turnTo(Math.PI/2);
    travelToLine();
    odometer.setX(12.6);
    odometer.setTheta(90);
    RIGHT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, 20), true);
    LEFT_MOTOR.rotate(-Navigation.convertDistance(project.WHEEL_RAD, 20), false);
    travelTo(0,0);
    LEFT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, odometer.getXYT()[2]), true);
    RIGHT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, odometer.getXYT()[2]), false);
  }

  /**
   * Travel to the front until the 2 light sensors poll a black line
   */
  private void travelToLine() {

    project.LEFT_MOTOR.setAcceleration(6000);
    project.RIGHT_MOTOR.setAcceleration(6000);
    
    project.LEFT_MOTOR.setSpeed(150);
    project.RIGHT_MOTOR.setSpeed(150);


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
          Thread.sleep(500);
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
          Thread.sleep(500);
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
  
  
  
  public void turnTo(double theta) {

    // ensures minimum angle for turning
    if (theta > Math.PI) {
        theta -= 2 * Math.PI;
    } else if (theta < -Math.PI) {
        theta += 2 * Math.PI;
    }

    // set Speed
    project.LEFT_MOTOR.setSpeed(100);
    project.RIGHT_MOTOR.setSpeed(100);

    // rotate motors at set speed

    // if angle is negative, turn to the left
    if (theta < 0) {
      project.LEFT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, -(theta * 180) / Math.PI), true);
      project.RIGHT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, -(theta * 180) / Math.PI), false);

    } else {
        // angle is positive, turn to the right
      project.LEFT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, (theta * 180) / Math.PI), true);
      project.RIGHT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, (theta * 180) / Math.PI), false);
    }
}
  
  
  public void travelTo(double x, double y) {
    double currx;
    double curry;
    double currTheta;
    double deltax;
    double deltay;
    
    currx = odometer.getXYT()[0];
    curry = odometer.getXYT()[1];

    deltax = x - currx;
    deltay = y - curry;

    // Calculate the angle to turn around
    currTheta = (odometer.getXYT()[2]) * Math.PI / 180;
    double mTheta = Math.atan2(deltax, deltay) - currTheta;

    double hypot = Math.hypot(deltax, deltay);

    // Turn to the correct angle towards the endpoint
    turnTo(mTheta);

    project.LEFT_MOTOR.setSpeed(150);
    project.RIGHT_MOTOR.setSpeed(150);

    project.LEFT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, hypot), true);
    project.RIGHT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, hypot), false);

    // stop vehicle
    project.LEFT_MOTOR.stop(true);
    project.RIGHT_MOTOR.stop(true);
  
  }
  
  public static int convertDistance(double radius, double distance) {
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
  public static int convertAngle(double radius, double width, double angle) {
    return convertDistance(radius, Math.PI * width * angle / 360.0);
  }
}
