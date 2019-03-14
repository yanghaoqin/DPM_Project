import java.io.FileWriter;
import java.io.PrintWriter;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.robotics.SampleProvider;

/**
 * A class to test the light sensor, record data, and output data into csv file. Program is able to
 * read multiple sets of data with multiple readings in each set. Set {@code i} and {@code j} for
 * information.
 * 
 * @author Raymond H. Yang
 * @version 1.0
 * @date 2019/03/12
 */
public class lineDetect {

  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------

  /**
   * The instance of the left wheel large EV3 motor. The left motor is connected to port A on the
   * EV3 brick.
   */
  private static final EV3LargeRegulatedMotor LEFT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * The instance of the right wheel large EV3 motor. The right motor is connected to port D on the
   * EV3 brick.
   */
  private static final EV3LargeRegulatedMotor RIGHT_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));


  /**
   * Port for light sensor
   */
  private static final Port LS_PORT = LocalEV3.get().getPort("S1");

  /**
   * The acceleration for the motors
   */
  private static final int ACCELERATION = 6000;

  /**
   * The target speed to achieve
   */
  private static final int SPEED = 250;

  /**
   * Threshold ratio for line detection
   */
  private static final double THRESHOLD = 0.75;

  /**
   * Number of lines to be passed before termination
   */
  private static final int LINES = 3;

  /**
   * Time in ms. Control sampling rate.
   */
  private static final int TIME = 30;

  /**
   * Number of times grid line crossed
   */
  private static int count;


  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  /**
   * Main method for test
   * 
   * @param args - None
   */
  public static void main(String args[]) throws Exception {

    System.out.println("Initializing...");

    // lines crossed
    count = 0;

    // set up filewriter
    FileWriter fw = new FileWriter("Data.csv");
    PrintWriter writer = new PrintWriter(fw);

    // sensor initialization
    @SuppressWarnings("resource")
    SampleProvider lsSample = (new EV3ColorSensor(LS_PORT)).getMode("Red");
    float[] lsData = new float[lsSample.sampleSize()];

    // take initial reading
    double initialReading = takeReading(lsSample, lsData, 100);

    System.out.println("Click any button to proceed or ESC to quit");

    // user ready
    if (Button.waitForAnyPress() != Button.ID_ESCAPE) {

      // start moving
      setDevice(ACCELERATION, SPEED);

      // while robot is moving, check if touch sensor activated
      while (LEFT_MOTOR.isMoving() || RIGHT_MOTOR.isMoving()) {

        // record time
        long initialTime = System.currentTimeMillis();

        // take 5 readings
        double intensity = takeReading(lsSample, lsData, 5);

        // record reading
        writer.println(intensity);

        // determine if line crossed
        if ((intensity / initialReading) < THRESHOLD) {
          Sound.beep();
          count += 1;
        }

        // record time
        long endTime = System.currentTimeMillis();

        // control sampling rate
        if ((endTime - initialTime) < TIME) {
          Thread.sleep(TIME - (endTime - initialTime));
        } else {
          // do nothing
        }

        // check for termination condition
        if (count == LINES) {

          // wait for 2 seconds
          Thread.sleep(2000);

          // stop motor
          stop();

          // display line count
          System.out.println("Lines: " + count);

          // press any button to exit
          Button.waitForAnyPress();
          fw.close();
          writer.close();
          System.exit(0);
        }
      }
    } else {
      // esc to quit
      fw.close();
      writer.close();
      System.exit(0);
    }


  }

  /**
   * Take readings of the color sensor
   * 
   * @param ls - SampleProvider instance
   * @param data - buffer to store data
   * @param num - number of readings
   * @return - average of amplified readings
   */
  private static double takeReading(SampleProvider ls, float[] data, int num) {
    double sum = 0;
    for (int i = 0; i < num; i++) {
      ls.fetchSample(data, 0);
      sum += data[0] * 100.0;
    }
    return (sum / num);
  }

  /**
   * Sets acceleration and speed for both motors. Start the robot.
   * 
   * @param acceleration - acceleration as integer
   * @param speed - speed as integer
   */
  private static void setDevice(int acceleration, int speed) {
    LEFT_MOTOR.setAcceleration(acceleration);
    RIGHT_MOTOR.setAcceleration(acceleration);
    LEFT_MOTOR.setSpeed(speed);
    RIGHT_MOTOR.setSpeed(speed);
    LEFT_MOTOR.forward();
    RIGHT_MOTOR.forward();
  }

  /**
   * Stop robot immediately
   */
  private static void stop() {
    LEFT_MOTOR.stop(true);
    RIGHT_MOTOR.stop(false);
  }
}
