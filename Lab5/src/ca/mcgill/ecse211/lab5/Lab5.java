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

  private static final TextLCD LCD = LocalEV3.get().getTextLCD();

  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  public static void main(String[] args) throws OdometerExceptions {

    int buttonChoice; // variable to record button clicked by user

    Odometer odometer = Odometer.getOdometer(LEFT_MOTOR, RIGHT_MOTOR, TRACK, WHEEL_RAD);

    Display odometryDisplay = new Display(LCD);

    // US sensor initialization
    @SuppressWarnings("resource")
    SensorModes usSensor = new EV3UltrasonicSensor(US_PORT);
    SampleProvider usDistance = usSensor.getMode("Distance");
    float[] usData = new float[usDistance.sampleSize()];

    // CS sensor initialization
    SensorModes csSensor = new EV3ColorSensor(CS_PORT);
    SampleProvider cs = csSensor.getMode("Red");
    float[] csData = new float[cs.sampleSize()];

    do {
      LCD.clear();
      LCD.drawString("< Left  |  Right >", 0, 0);
      LCD.drawString("        |         ", 0, 1);
      LCD.drawString("Falling |  Rising ", 0, 2);
      LCD.drawString(" Edge   |   Edge  ", 0, 3);
      buttonChoice = Button.waitForAnyPress();
    } while (buttonChoice != Button.ID_LEFT && buttonChoice != Button.ID_RIGHT
        && buttonChoice != Button.ID_ESCAPE);

    if (buttonChoice == Button.ID_LEFT || buttonChoice == Button.ID_RIGHT) {
      // Falling Edge or Rising Edge

      UltrasonicLocalizer usloc = new UltrasonicLocalizer(buttonChoice, usDistance, usData, odometer);
      
      Thread odoThread = new Thread(odometer);
      Thread odoDisplayThread = new Thread(odometryDisplay);
      Thread uslocThread = new Thread(usloc);
      
      odoThread.start();
      odoDisplayThread.start();
      uslocThread.start();
      //TODO: REMOVE WAIT FOR USER INPUT FROM LAB 4
      if (Button.waitForAnyPress() == Button.ID_ESCAPE) { // wait for user confirmation
        System.exit(0);
      } else {
        LightLocalizer lightloc = new LightLocalizer(cs, csData, odometer);
        Thread lightlocThread = new Thread(lightloc);
        lightlocThread.start();
      }
      // LAB 4 CODE STOPS EXECUTING HERE
      

    } else {
      System.exit(0);
    }

    // keep the program from ending unless esc button is pressed
    while (Button.waitForAnyPress() != Button.ID_ESCAPE) {

    }
    System.exit(0); // exit program after esc pressed
  }
}
