package ca.mcgill.ecse211.lab5;

// non-static imports
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import java.util.Arrays;
// static imports
import static ca.mcgill.ecse211.lab5.Lab5.LEFT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.RIGHT_MOTOR;
import static ca.mcgill.ecse211.lab5.Lab5.TRACK;
import static ca.mcgill.ecse211.lab5.Lab5.WHEEL_RAD;

public class UltrasonicLocalizer {

  private static final double THRESHOLD = 20;
  private static final double MARGIN = 7.5;
  private static final int TURN_SPEED = 100;
  private static final int SMOOTH_ACCELERATION = 500;
  private static final double TURN_CIRCLE = 360.0;
  private static final int INITIAL_ANGLE = 0;
  private static final int HALF_CIRCLE = 180;
  private static final int FULL_CIRCLE = 360;

  private static final int FALLING_EDGE = Button.ID_LEFT;
  private static final int RISING_EDGE = Button.ID_RIGHT;

  private int type;

  private Odometer odometer;
  private double[] odoData;

  private SampleProvider usDistance;
  private float[] usData;

  public UltrasonicLocalizer(int buttonChoice, SampleProvider usDistance, float[] usData, Odometer odometer) {
    this.odometer = odometer;
    this.usData = usData;
    this.usDistance = usDistance;
    this.type = buttonChoice;
    LEFT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
    RIGHT_MOTOR.setAcceleration(SMOOTH_ACCELERATION);
    LEFT_MOTOR.setSpeed(TURN_SPEED);
    RIGHT_MOTOR.setSpeed(TURN_SPEED);
  }

  public void localize() {

    double dTheta = 0;

    if (type == FALLING_EDGE) {
      // falling edge
      dTheta = fallingEdge();
    } else if (type == RISING_EDGE) {
      // rising edge
      dTheta = risingEdge();
    }

    LEFT_MOTOR.stop(true);
    RIGHT_MOTOR.stop(false);

    odometer.setTheta(odometer.getXYT()[2] + dTheta);

    Sound.beep();

    turnTo(0);

  }

  /**
   * Falling Edge
   */
  private double fallingEdge() {

    double alpha, beta;

    try {
      Thread.sleep(2000);
    } catch (Exception e) {

    }

    while (medianFilter() > 25) {
      LEFT_MOTOR.backward();
      RIGHT_MOTOR.forward();
    }

    LEFT_MOTOR.stop(true);
    RIGHT_MOTOR.stop(false);

    Sound.beep();

    alpha = odometer.getXYT()[2];

    while (medianFilter() < 40) {
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }
    Sound.beep();

    while (medianFilter() > 25) {
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }

    Sound.beep();

    beta = odometer.getXYT()[2];

    return correctAngle(alpha, beta);
  }

  /**
   * 
   */
  private double risingEdge() {

    double alpha, beta;

    try {
      Thread.sleep(2000);
    } catch (Exception e) {

    }

    while (medianFilter() < 40) {
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.backward();
    }

    LEFT_MOTOR.stop(true);
    RIGHT_MOTOR.stop(false);

    Sound.beep();

    alpha = odometer.getXYT()[2];

    while (medianFilter() > 25) {
      LEFT_MOTOR.backward();
      RIGHT_MOTOR.forward();
    }
    Sound.beep();

    while (medianFilter() < 35) {
      LEFT_MOTOR.backward();
      RIGHT_MOTOR.forward();
    }
    Sound.beep();

    beta = odometer.getXYT()[2];

    return correctAngle(alpha, beta);

  }

  private double correctAngle(double alpha, double beta) {
    if (alpha > beta) {
      return 225.0 - (alpha + beta) / 2.0;
    } else {
      return 45.0 - (alpha + beta) / 2.0;
    }

  }

  /**
   * Turns the robot at a fixed position to face the next destination, changes the robot heading.
   * The minimum angle is calculated by determining whether the clockwise angle Theta is greater
   * than half of a full circles
   * 
   * @param Theta - the clockwise angle to turn from Theta = 0
   */
  public void turnTo(double Theta) {

    // ensure angle is positive and within 360
    double minTheta = ((Theta - odometer.getXYT()[2]) + FULL_CIRCLE) % FULL_CIRCLE;

    if (minTheta > INITIAL_ANGLE && minTheta <= HALF_CIRCLE) {
      // angle is already minimum angle, robot should turn clockwise
      RIGHT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), true);
      LEFT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), false);
    } else if (minTheta > HALF_CIRCLE && minTheta < FULL_CIRCLE) {
      // angle is not minimum angle, robot should turn counter-clockwise to the
      // complementary angle of a full circle 360 degrees
      minTheta = FULL_CIRCLE - minTheta;
      RIGHT_MOTOR.rotate(convertAngle(WHEEL_RAD, TRACK, minTheta), true);
      LEFT_MOTOR.rotate(-convertAngle(WHEEL_RAD, TRACK, minTheta), false);
    }

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
    double[] arr = new double[3];
    for (int i = 0; i < 3; i++) {
      this.usDistance.fetchSample(usData, 0);
      arr[i] = usData[0] * 100.0;
    }
    Arrays.sort(arr);

    if (arr[1] > 255) {
      return 255;
    }

    return arr[1];
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
}
