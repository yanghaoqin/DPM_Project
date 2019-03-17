/*
 * OdometryCorrection.java
 */
package ca.mcgill.ecse211.odometer;

import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.lcd.TextLCD;
import lejos.hardware.port.Port;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.SensorModes;
import lejos.robotics.SampleProvider;

public class OdometryCorrection implements Runnable {
  private static final long CORRECTION_PERIOD = 10;
  private static final double TILE_LENGTH = 30.48;
  private static int horizontalL; // counter for black line in x-axis
  private static int verticalL; // counter for black line in y-axis


  private Odometer odometer;
  private float[] leftlsData;
  private float[] rightlsData;
  private static Port leftPort = LocalEV3.get().getPort("S1");
  private static Port rightPort = LocalEV3.get().getPort("S2");
  private SampleProvider leftcolor;
  private SampleProvider rightcolor;
  private SensorModes leftlightSensor;
  private SensorModes rightlightSensor;
  private double correctionInX;
  private double correctionInY;
  private static final TextLCD lcd = LocalEV3.get().getTextLCD();

  private double xHolder;
  private double yHolder;

  /**
   * This is the default class constructor. An existing instance of the odometer is used. This is to
   * ensure thread safety.
   * 
   * @throws OdometerExceptions
   */
  public OdometryCorrection() throws OdometerExceptions {

    this.odometer = Odometer.getOdometer();
    this.leftlightSensor = new EV3ColorSensor(leftPort);
    this.rightlightSensor = new EV3ColorSensor(rightPort);
    this.leftcolor = leftlightSensor.getMode("Red");
    this.rightcolor = rightlightSensor.getMode("Red");
    this.leftlsData = new float[leftlightSensor.sampleSize()];
    this.rightlsData = new float[rightlightSensor.sampleSize()];
    horizontalL = 0;
    verticalL = 0;
    correctionInX = 0.0;
    correctionInY = 0.0;

  }

  /**
   * Here is where the odometer correction code should be run.
   * 
   * @throws OdometerExceptions
   */
  // run method (required for Thread)
  public void run() {
    long correctionStart, correctionEnd;

    float leftcolorIntensity = 0;
    float rightcolorIntensity = 0;
    
    double initVal = TILE_LENGTH / 2;
    odometer.setXYT(-initVal, -initVal, 0);
    double curX = -initVal;
    double curY = -initVal;

    boolean leftline = false;
    boolean rightline = false;
    
    double[] lastline = {0, 0, 0};
    
    while (true) {

      // odometer.setXYT(0, 0, 0);
      correctionStart = System.currentTimeMillis();

      // Trigger correction (When do I have information to correct?)
      // Calculate new (accurate) robot position
      // Update odometer with new calculated (and more accurate) values

      leftcolor.fetchSample(leftlsData, 0);
      leftcolorIntensity = leftlsData[0];

      if (leftcolorIntensity < 0.38) { // black lines reflect low intensity light
        Sound.beep();
        if(!rightline) {
          leftline = true;  
        }
        
        
        double[] odoData = odometer.getXYT(); // using get method from OdometerData
        double theta = odoData[2];



        if (theta < 10 || theta > 350) { // theta around 0 degrees; moving in positive y direction
          if (verticalL == 0) {
            curY += (TILE_LENGTH / 2);
            verticalL++;

          } else {
            curY += TILE_LENGTH;
          }
          odometer.setY(curY);
        }

        else if (theta < 100 && theta >= 80) { // theta around 90degrees; moving in positive x
                                               // direction
          if (horizontalL == 0) {
            curX += (TILE_LENGTH / 2);
            curY = odometer.getXYT()[1];
            horizontalL++;

          } else {
            curX += TILE_LENGTH;
          }
          odometer.setX(curX);

        }

        else if (theta < 190 && theta >= 170) { // theta around 180 degrees; moving in negative y
                                                // direction
          if (verticalL == 1) {
            curY -= (TILE_LENGTH / 2);
            verticalL--;
            curX = odometer.getXYT()[0];


          } else {
            curY -= TILE_LENGTH;
          }
          odometer.setY(curY);

        }

        else if (theta < 280 && theta >= 260) { // theta around 270 degrees; moving in negative x
                                                // direction
          if (horizontalL == 1) {
            horizontalL--;
            curX -= (TILE_LENGTH / 2);
            curY = odometer.getXYT()[1];


          } else {
            curX -= TILE_LENGTH;
          }
          odometer.setX(curX);

        }
        // System.out.println("X: " + curX + "Y: " + curY);

        // odometer.setXYT(curX, curY, theta);

        if(!rightline) {
          lastline = odometer.getXYT();  
        }
        else {
          
        }
        
        
        rightline = false;
        // this ensure the odometry correction occurs only once every period
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
          } catch (InterruptedException e) {
            // there is nothing to be done here
          }
        }
      }
      
      if (rightcolorIntensity < 0.38) { // black lines reflect low intensity light
        Sound.beep();
        if(!leftline) {
          rightline = true;
        }
        
        
        double[] odoData = odometer.getXYT(); // using get method from OdometerData
        double theta = odoData[2];



        if (theta < 10 || theta > 350) { // theta around 0 degrees; moving in positive y direction
          if (verticalL == 0) {
            curY += (TILE_LENGTH / 2);
            verticalL++;

          } else {
            curY += TILE_LENGTH;
          }
          odometer.setY(curY);
        }

        else if (theta < 100 && theta >= 80) { // theta around 90degrees; moving in positive x
                                               // direction
          if (horizontalL == 0) {
            curX += (TILE_LENGTH / 2);
            curY = odometer.getXYT()[1];
            horizontalL++;

          } else {
            curX += TILE_LENGTH;
          }
          odometer.setX(curX);

        }

        else if (theta < 190 && theta >= 170) { // theta around 180 degrees; moving in negative y
                                                // direction
          if (verticalL == 1) {
            curY -= (TILE_LENGTH / 2);
            verticalL--;
            curX = odometer.getXYT()[0];


          } else {
            curY -= TILE_LENGTH;
          }
          odometer.setY(curY);

        }

        else if (theta < 280 && theta >= 260) { // theta around 270 degrees; moving in negative x
                                                // direction
          if (horizontalL == 1) {
            horizontalL--;
            curX -= (TILE_LENGTH / 2);
            curY = odometer.getXYT()[1];


          } else {
            curX -= TILE_LENGTH;
          }
          odometer.setX(curX);

        }
        // System.out.println("X: " + curX + "Y: " + curY);

        // odometer.setXYT(curX, curY, theta);

        if(!leftline) {
          lastline = odometer.getXYT();
        }
        else {
           double correctionX = odometer.getXYT()[0] - lastline[0];
           double correctionY = odometer.getXYT()[1] - lastline[1];
           if(correctionX > 0.1 || correctionY > 0.1) {
             odometer.setTheta(Math.atan2(correctionX, correctionY) + 90);
             }
           }

        leftline = false;
        // this ensure the odometry correction occurs only once every period
        correctionEnd = System.currentTimeMillis();
        if (correctionEnd - correctionStart < CORRECTION_PERIOD) {
          try {
            Thread.sleep(CORRECTION_PERIOD - (correctionEnd - correctionStart));
          } catch (InterruptedException e) {
            // there is nothing to be done here
          }
        }
      }
      
    }
  }
}
