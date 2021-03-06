package ca.mcgill.ecse211.lab5;

// non-static imports
import lejos.hardware.Button;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;

/**
 * The main class that controls all the threads and execution of functions. Motors and sensors are
 * initialized here. Constants are declared here.
 *
 * @author Team 23
 */
public class Lab5 {

  /**
   * Size of one tile in cm
   */
  public static final double TILE = 30.48;

  /**
   * A constant factor that can be applied to convert angle units in radians to degrees
   */
  public static final double TO_DEG = 180.0 / Math.PI;

  /**
   * A constant factor that can be applied to convert angular units in degrees to radians
   */
  public static final double TO_RAD = Math.PI / 180.0;

  /**
   * The radius (in cm) of the left/right wheels of the EV3 robot.
   */
  public static final double WHEEL_RAD = 2.15;

  /**
   * The width (in cm) of the robot measured from the center of the left wheel to the center of the
   * right wheel
   */
  public static final double TRACK = 13.3;

  /**
   * A value for motor acceleration that prevents the wheels from slipping on the demo floor by
   * accelerating and decelerating slowly
   */
  public static final int SMOOTH_ACCELERATION = 500;

  /**
   * Specifies the speed of the left and right EV3 Large motors
   */
  public static final int SPEED = 100;

  /**
   * The heading/Theta value of the robot initially
   */
  public static final int INITIAL_ANGLE = 0;

  /**
   * A revolution of half of a circle in degrees
   */
  public static final int HALF_CIRCLE = 180;

  /**
   * A full revolution of a circle in degrees
   */
  public static final int FULL_CIRCLE = 360;

  // r, g, b in order
  public static final double[] BLUE_COLOR = {0.19, 0.40, 0.87}; // value of blue colour
  public static final double[] GREEN_COLOR = {0.35, 0.85, 0.39}; // value of green colour
  public static final double[] YELLOW_COLOR = {0.85, 0.52, 0.09}; // value of yellow colour
  public static final double[] RED_COLOR = {0.98, 0.24, 0.07}; // value of red colour

  /**
   * The instance of the left wheel large EV3 motor. The left motor is connected to port A on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The instance of the right wheel large EV3 motor. The right motor is connected to port D on the
   * EV3 brick.
   */
  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * Whether robot is currently in color detection
   */
  public static boolean isColorDetection;

  /**
   * The instance of the medium motor that turns the sensor. The motor is connected to port B on the
   * EV3 brick.
   */
  public static final EV3MediumRegulatedMotor SENSOR_MOTOR =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("B"));

  /**
   * Port for ultrasonic sensor.
   */
  private static final Port US_PORT = LocalEV3.get().getPort("S2");

  /**
   * Port for color sensor for localization.
   */
  private static final Port CS_PORT = LocalEV3.get().getPort("S4");

  /**
   * Port for color sensor for can classification.
   */
  private static final Port COLOR_PORT = LocalEV3.get().getPort("S3");

  /**
   * LCD display instance.
   */
  private static final TextLCD LCD = LocalEV3.get().getTextLCD();

  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  /**
   * The main method. This method is used to start threads and execute the main function of the
   * robot. Mainly localization and search.
   * 
   * @param args - arguments to pass in
   * @throws OdometerExceptions - multiple odometer instances
   */
  public static void main(String[] args) throws OdometerExceptions {

    // variable to record button clicked by user
    int buttonChoice;

    // set up odometer
    Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

    // set up display
    Display EV3Display = new Display(LCD);

    // US sensor initialization
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    // light sensor initialization, for can classification
    @SuppressWarnings("resource")
    SensorModes lightSensor = new EV3ColorSensor(COLOR_PORT);
    SampleProvider lightColor = ((EV3ColorSensor) lightSensor).getRGBMode();
    float[] lightData = new float[3];

    // color sensor for localization initialization
    @SuppressWarnings("resource")
    SensorModes csSensor = new EV3ColorSensor(CS_PORT);
    SampleProvider cs = csSensor.getMode("Red");
    float[] csData = new float[cs.sampleSize()];

    // Option to choose light sensor test or start search routine
    do {
      LCD.clear();
      LCD.drawString("< Left  |  Right >", 0, 0);
      LCD.drawString("        |         ", 0, 1);
      LCD.drawString("   LS   |  Start  ", 0, 2);
      LCD.drawString("  TEST  |  Search ", 0, 3);
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ESCAPE);

    // light sensor testing
    if (buttonChoice == Button.ID_LEFT) {

      // set status
      isColorDetection = true;

      // initialize threads and instances
      Thread odoThread = new Thread(odometer);
      Thread displayThread = new Thread(EV3Display);
      CanCalibrator cc = new CanCalibrator(lightColor, lightData);
      ColorDetection cd = new ColorDetection(usDistance, usData, lightColor, lightData, LCD);

      // start threads for odometer, display, and color detection
      odoThread.start();
      displayThread.start();
      cd.start();

      // loop to continue calibrate
      while (buttonChoice != Button.ID_ESCAPE) {
        cc.Calibrate();
      }

      // exit system after esc pressed
      System.exit(0);

    } else {

      // in search mode
      isColorDetection = false;

      Thread odoThread = new Thread(odometer);
      Thread displayThread = new Thread(EV3Display);

      // option to choose localization mode
      do {
        LCD.clear();
        LCD.drawString("< Left  |  Right >", 0, 0);
        LCD.drawString("        |         ", 0, 1);
        LCD.drawString("Falling |  Rising ", 0, 2);
        LCD.drawString(" Edge   |   Edge  ", 0, 3);
        buttonChoice = Button.waitForAnyPress();
      } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
          && buttonChoice != Button.ID_ESCAPE);

      // clear display
      LCD.clear();

      // start threads
      odoThread.start();
      displayThread.start();

      // ultrasonic localization
      UltrasonicLocalizer UL = new UltrasonicLocalizer(buttonChoice, usDistance, usData, odometer);
      UL.localize();

      // light sensor localization
      LightLocalizer LL = new LightLocalizer(cs, csData, odometer);
      LL.localize();

      // search method
      Search search = new Search(odometer, usDistance, usData, lightColor, lightData, cs, csData);
      Thread searchThread = new Thread(search);
      searchThread.start();

      // exit when esc pressed
      while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
      }
      System.exit(0); // exit program after esc pressed
    }
  }
}
