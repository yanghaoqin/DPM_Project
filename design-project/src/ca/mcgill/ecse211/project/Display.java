package ca.mcgill.ecse211.project;

import java.text.DecimalFormat;
import ca.mcgill.ecse211.project.CanCalibrator;
import ca.mcgill.ecse211.project.project;
import ca.mcgill.ecse211.project.Search;
import ca.mcgill.ecse211.odometer.Odometer;
import ca.mcgill.ecse211.odometer.OdometerExceptions;
import lejos.hardware.lcd.TextLCD;

/*this class will contain all the methods relative to display*/
/**
 * This class is used to display the content of the odometer variables (x, y, Theta). This class
 * include constants and class variables that specifies the display behaviours of the LCD on the EV3
 * platform. The class implements the {@code Runnable} interface and has a {@code run()} method
 */
public class Display implements Runnable {

  // -----------------------------------------------------------------------------
  // Constants
  // -----------------------------------------------------------------------------

  // controls LCD refresh frequency, each set of value displays for 25 ms
  private final long DISPLAY_PERIOD = 25;

  // a time limit to stop the display program if it takes too long
  private long timeout = Long.MAX_VALUE;

  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------

  // the odometer object
  private Odometer odo;

  // the display LCD screen object
  private TextLCD lcd;

  // a double array used to store the position data X,Y, and Theta
  private double[] position;

  // -----------------------------------------------------------------------------
  // Constructors
  // -----------------------------------------------------------------------------

  /**
   * This is the class constructor which initializes the odometer and the LCD display.
   * 
   * @param lcd - the LCD display instance
   * @throws OdometerExceptions - throws odometer exceptions if fails to get odometer
   */
  public Display(TextLCD lcd) throws OdometerExceptions {
    odo = Odometer.getOdometer(); // calls method to pass odometer handle
    this.lcd = lcd; // initializes LCD object
  }

  /**
   * This is the overloaded class constructor, initializes odometer and LCD but also with a updated
   * time limit. Gives user option of changing time limit.
   * 
   * @param lcd - The LCD display object
   * @param timeout - Time limit set by user for display program
   * @throws OdometerExceptions - throws odometer exceptions if fails to get odometer
   */
  public Display(TextLCD lcd, long timeout) throws OdometerExceptions {
    odo = Odometer.getOdometer(); // calls method to pass odometer handle
    this.timeout = timeout; // sets user defined time limit
    this.lcd = lcd; // initializes LCD object
  }

  // -----------------------------------------------------------------------------
  // Public Methods
  // -----------------------------------------------------------------------------

  /**
   * This method sets up the LCD display for the robot and updates the position values at a specific
   * rate. The output is formatted to 2 decimal places
   */
  public void run() {

    // clear EV3 display
    lcd.clear();

    // stores method start time and finish time
    long updateStart, updateEnd;

    // record current time
    long tStart = System.currentTimeMillis();

    do {

      // record current time
      updateStart = System.currentTimeMillis();

      // Retrieve x, y and Theta information
      position = odo.getXYT();

      DecimalFormat numberFormat = new DecimalFormat("######0.00");

      // Print red, green, and blue information on display with specific format
      // Format:
      // --------------------
      // R: #.## + #.##
      // G: #.## + #.##
      // B: #.## + #.##
      // --------------------
      if (project.isColorDetection == true) {
          lcd.drawString("RED: " + numberFormat.format(CanCalibrator.mean[0]) + " +"
                    + numberFormat.format(CanCalibrator.standard_deviation[0]), 0, 3);
                lcd.drawString("GREEN: " + numberFormat.format(CanCalibrator.mean[1]) + " +"
                    + numberFormat.format(CanCalibrator.standard_deviation[1]), 0, 4);
                lcd.drawString("BLUE: " + numberFormat.format(CanCalibrator.mean[2]) + " +"
                    + numberFormat.format(CanCalibrator.standard_deviation[2]), 0, 5);
      } else {

        // Print x, y, theta, red, green, and blue information on display with specific format
        // Format:
        // --------------------
        // x: #.##
        // y: #.##
        // T: #.##
        // R: #.## + #.##
        // G: #.## + #.##
        // B: #.## + #.##
        // --------------------
        
        lcd.drawString("X: " + numberFormat.format(position[0]), 0, 0);
        lcd.drawString("Y: " + numberFormat.format(position[1]), 0, 1);
        lcd.drawString("T: " + numberFormat.format(position[2]), 0, 2);
      //  lcd.drawString("distance: " + numberFormat.format(Search.distDisplay), 0, 6); //TODO: FIX WHEN MAKING NEW SEARCH ALGORITHM
      }
      
      // this ensures that the data is updated only once every period
      // record current time
      updateEnd = System.currentTimeMillis();

      // check if time elapsed is shorter than required
      if (updateEnd - updateStart < DISPLAY_PERIOD) {
        try {
          // suspend thread until time period satisfied
          Thread.sleep(DISPLAY_PERIOD - (updateEnd - updateStart));
        } catch (InterruptedException e) {
          e.printStackTrace();
        }
      }
      // if loop takes too much time, stop program
    } while ((updateEnd - tStart) <= timeout);

  }

}
