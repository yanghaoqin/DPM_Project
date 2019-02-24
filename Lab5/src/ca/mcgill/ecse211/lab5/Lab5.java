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

public class Lab5 {

  public static final double WHEEL_RAD = 2.15;
  public static final double TRACK = 13.3;
  public static final double TILE = 30.48;

  public static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  public static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The instance of the medium EV3 motor that controls the turning of the ultrasonic sensor. The
   * motor is connected to port C on the EV3 brick.
   */

  public static final EV3MediumRegulatedMotor SENSOR_MOTOR =
      new EV3MediumRegulatedMotor(LocalEV3.get().getPort("C"));

  private static final Port US_PORT = LocalEV3.get().getPort("S1");

  private static final Port CS_PORT = LocalEV3.get().getPort("S2");

  private static final Port COLOR_PORT = LocalEV3.get().getPort("S3");

  private static final TextLCD LCD = LocalEV3.get().getTextLCD();

  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice; // variable to record button clicked by user

    Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

    Display EV3Display = new Display(LCD);

    // US sensor initialization
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    @SuppressWarnings("resource")
    SensorModes lightSensor = new EV3ColorSensor(COLOR_PORT);
    SampleProvider lightColor = ((EV3ColorSensor) lightSensor).getRGBMode();
    float[] lightData = new float[3];

    // CS sensor initialization
    @SuppressWarnings("resource")
    SensorModes csSensor = new EV3ColorSensor(CS_PORT);
    SampleProvider cs = csSensor.getMode("Red");
    float[] csData = new float[cs.sampleSize()];

    do {
      LCD.clear();
      LCD.drawString("< Left  |  Right >", 0, 0);
      LCD.drawString("        |         ", 0, 1);
      LCD.drawString("   LS   |  Start  ", 0, 2);
      LCD.drawString("  TEST  |  Search ", 0, 3);
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ESCAPE && buttonChoice != Button.ID_ENTER);

    if (buttonChoice == Button.ID_LEFT) {
      // LS TESTING

      Thread odoThread = new Thread(odometer);
      Thread odoDisplayThread = new Thread(EV3Display);
      CanCalibrator cc = new CanCalibrator(lightColor, lightData);

      // the rgb for the color of the target can
      double[] target = {0.6, 0.8, 0.58};

      odoThread.start();
      odoDisplayThread.start();

      while (buttonChoice != Button.ID_ESCAPE) {
        cc.Calibrate(target); // take readings
      }

      System.exit(0); // terminate program

    } else {

    }


    // -----------------------------------------------------------------------------
    // Lab 5 testing light sensor
    // -----------------------------------------------------------------------------

    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {
    }

    // -----------------------------------------------------------------------------
    // Lab 5 testing light sensor
    // -----------------------------------------------------------------------------

    System.exit(0); // exit program after esc pressed
  }

  /**
   * initial readings taken (100 times) and the average is used to distinguish between the wooden
   * board and the black line. Each reading is amplified to enhance the sensitivity of the sensor
   * 
   * @return
   */
}
