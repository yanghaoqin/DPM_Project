package ca.mcgill.ecse211.project;

import ca.mcgill.ecse211.project.MotorSweep;
import lejos.hardware.Sound;
import lejos.hardware.lcd.TextLCD;
import lejos.robotics.SampleProvider;
import ca.mcgill.ecse211.odometer.*;
import static ca.mcgill.ecse211.project.project.LEFT_MOTOR;
import static ca.mcgill.ecse211.project.project.RIGHT_MOTOR;
import static ca.mcgill.ecse211.project.project.SENSOR_MOTOR;
import static ca.mcgill.ecse211.project.project.TILE;
import java.util.Arrays;


/**
 * This class is responsible for the searching section of the project. It will enable the robot to search for cans in the search zone.
 * 
 * @author Erica De Petrillo
 * */
public class Search extends Thread{

   /* private static final int URx = 10; //upper right corner of search zone x coord (modify if needed)
    private static final int URy = 10; //upper right corner of search zone y coord (modify if needed)
    private static final int LLx = 3; //lower left corner x coord (modify if needed)
    private static final int LLy = 3; //lower left corner y coord (modify if needed)*/
    private Odometer odo;
    private SampleProvider usDistance;
    private float[] usData;
    private static final float SPEED = 150; //TODO: TWEAK
    private static final float CAN = 30; //distance recorded by the us sensor at which there must be a can, TODO: TWEAK
 //   private static final double WHEEL_RAD = 2.15; //TODO: TWEAK
  //  private static final double TRACK = 20; //TODO: TWEAK
    private static final double CAN_CLOSE = 10; //closer distance at which there must be a can. robot must stop as to not hit can. TODO: TWEAK
    private float angleTacho; //tacho count for angle at which can detected
    private boolean isRed; //TODO: CHANGE WHEN NEEDED true if red team, false if green team
    private boolean isForward; //true if robot going towards end of search zone, false if robot going towards start of search zone
    private SampleProvider lightColor; //to call can colour id
    private float[] lightData; //to call colour id
    private TextLCD lcd; //to call colour id
   /**
    * The constructor for the Search class.
    * 
    * @param odo the Odometer of the robot
    * @param usDistance the SampleProvider for the ultrasonic sensor
    * @param usData the array in which the samples can be stored
   * @param lcd 
   * @param lightData 
   * @param lightColor 
    */
    public Search(Odometer odo, SampleProvider usDistance, float[] usData, SampleProvider lightColor, float[] lightData, TextLCD lcd) {
      this.odo = odo;
      this.usDistance = usDistance;
      this.usData = usData;
      this.lightColor = lightColor;
      this.lightData = lightData;
      this.lcd = lcd;
    }
    
    
    /**
     * This method defines the search algorithm that the robot must follow to find cans.
     * The robot advances in a straight line until the vertical end of the search zone while sweeping with the ultrasonic sensor.
     * If a can is found, the robot goes toward that can to then identify, weight, and handle it.
     * If not, the robot reaches the end of the vertical limit of the search zone and turns towards the next search lane.
     * The same process starts again, until a can is found, or the horizontal end of the search zone is reached.
     */
    public void run() {
        
      MotorSweep motorSweep = new MotorSweep(SENSOR_MOTOR);
      Thread motorThread = new Thread(motorSweep);   
      motorSweep.startSensor();
      motorThread.start();
      
      
      float angle; //angle at which the can is located
      
      boolean canFound = false; //turns to true when a can is found
      
   
      //LET'S ASSUME WE START FROM LL CORNER
      
      LEFT_MOTOR.setSpeed(SPEED);
      RIGHT_MOTOR.setSpeed(SPEED);
      LEFT_MOTOR.forward();
      RIGHT_MOTOR.forward();
      isForward = true;
      
      while (true) { //loop for whole search
      /*  double URY = project.zone_UR_y * TILE;
        double LLY = project.zone_LL_y * TILE;
        double URX = project.zone_UR_x * TILE;*/

      /*  System.out.println("Y: " + Y);   
        System.out.println("X: " + X);   
        System.out.println("URY: " + URY);   
        System.out.println("LLY: " + LLY);   
        System.out.println("URX: " + URX);   */

        while ((odo.getXYT()[1] < project.zone_UR_y*TILE) //TODO: REPLACE HARDCODE BY WIFI CLASS VARIABLES
            && (odo.getXYT()[1] >= project.zone_LL_y*TILE) 
            && (odo.getXYT()[0] < project.zone_LL_x*TILE)) { //while still in search zone
          double distance = medianFilter();
          if (distance < CAN) { //can is detected
            motorSweep.stopSensor(); //motor stops sweeping
            angleTacho = SENSOR_MOTOR.getTachoCount();
            angle = angleTacho - motorSweep.straightTacho; //find angle at which robot must rotate to keep going straight to find can
                        
            LEFT_MOTOR.stop(true);
            RIGHT_MOTOR.stop();
            
            if (isForward) {
              RIGHT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, angle /2), true);
              LEFT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, angle /2), false);
            }
            else {
            RIGHT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, angle /2), true);
            LEFT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, angle /2), false);
            }
            //turns to that angle
            
            LEFT_MOTOR.setSpeed(SPEED);
            RIGHT_MOTOR.setSpeed(SPEED);
            LEFT_MOTOR.forward();
            RIGHT_MOTOR.forward(); //now robot moving forward towards can
            SENSOR_MOTOR.setSpeed(SPEED);
            SENSOR_MOTOR.forward();
            
            while (true) {    
              distance = medianFilter();
              if (distance < CAN_CLOSE) { //can is detected
                LEFT_MOTOR.stop(true);
                RIGHT_MOTOR.stop(); //robot stops when can detected
                
                ColorDetection colDet = new ColorDetection(usDistance, usData, lightColor, lightData, lcd);
                int colorIndex = colDet.rotateSensorDetect();
                
                if (colorIndex == 0) {
                  lcd.drawString("Red Can", 0, 1);
                } else if (colorIndex == 1) {
                  lcd.drawString("Green Can", 0, 1);
                } else if (colorIndex == 2) {
                  lcd.drawString("Blue Can", 0, 1);
                } else if (colorIndex == 3) {
                  lcd.drawString("Yellow Can", 0, 1);
                }//TODO: YUNHAO, PLS MODIFY THE COLOUR CODE
                
                if (colorIndex == 1) { //only fr beta demo //TODO: FIX HARDCODE
                  Sound.beep();
                  Sound.beep();
                  Sound.beep();
                  Sound.beep();
                  Sound.beep();
                  Sound.beep();
                  Sound.beep();
                  Sound.beep();
                  Sound.beep();
                  Sound.beep();
                  //for beta demo, we must get away from can so:
                  LEFT_MOTOR.setSpeed(SPEED);
                  RIGHT_MOTOR.setSpeed(SPEED);
                  LEFT_MOTOR.backward();
                  RIGHT_MOTOR.backward(); //now robot moving away from can
                  
                  while (true) {
                    distance = medianFilter();
                    if (distance >= CAN_CLOSE) { 
                      LEFT_MOTOR.stop(true);
                      RIGHT_MOTOR.stop(); //robot stops when can detected
                      LEFT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, angle), true);
                      RIGHT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, angle), false);
                      break; //at this point robot should be far enough from can to be able to travel to upper right corner of search
                    }
                  }
                  return; //exit search class, return to project class
                } //we found the right can
                else { //not the right can
                  LEFT_MOTOR.setSpeed(SPEED);
                  RIGHT_MOTOR.setSpeed(SPEED);
                  LEFT_MOTOR.backward();
                  RIGHT_MOTOR.backward(); //now robot moving away from can
                  
                  while (true) {
                    distance = medianFilter();
                    if (distance >= CAN_CLOSE) { 
                      LEFT_MOTOR.stop(true);
                      RIGHT_MOTOR.stop(); //robot stops when can detected
                      LEFT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, angle), true);
                      RIGHT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, angle), false);
                    
                      LEFT_MOTOR.setSpeed(SPEED);
                      RIGHT_MOTOR.setSpeed(SPEED);
                      LEFT_MOTOR.forward();
                      RIGHT_MOTOR.forward();
                      LEFT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, 10)); //TODO: TWEAK 10
                      RIGHT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, 10));
                      
                      break; //at this point robot should be back on search track to look for next can, exits the while loop necessary for backtracking
                    }

                  }
                  break; //exits the can found while loop
                }

              }

            }

          }

        }
        //this while loop is exited when robot not in search zone anymore
        if (odo.getXYT()[0] <= project.zone_LL_x*TILE) { //robot still in search zone --> we need to make it change direction
          LEFT_MOTOR.stop(true);
          RIGHT_MOTOR.stop(); 
          
          if (isForward) { //must turn right
           RIGHT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, 93), true); //turns 90 degrees right
           LEFT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, 93), false);
           
           RIGHT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, TILE), true);
           LEFT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, TILE), false); //moves to next gridline
           
           RIGHT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, 93), true); //turns 180 degrees right
           LEFT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, 93), false);
           
           isForward = false; //since we changed direction
          }
         else { //must turn left
           LEFT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, 93), true); //turns 90 degrees left
           RIGHT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, 93), false);
           
           RIGHT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, TILE), true);
           LEFT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, TILE), false); //moves to next gridline
           
           LEFT_MOTOR.rotate(-convertAngle(project.WHEEL_RAD, project.TRACK, 93), true); //turns 180 degrees left
           RIGHT_MOTOR.rotate(convertAngle(project.WHEEL_RAD, project.TRACK, 93), false);
           
           isForward = true; //since we changed direction
         }  
         LEFT_MOTOR.setSpeed(SPEED);
         RIGHT_MOTOR.setSpeed(SPEED);
         LEFT_MOTOR.forward();
         RIGHT_MOTOR.forward();
         
         LEFT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, 10), true); //TODO: TWEAK 10
         RIGHT_MOTOR.rotate(convertDistance(project.WHEEL_RAD, 10), false);
         
         LEFT_MOTOR.stop(true);
         RIGHT_MOTOR.stop(); 
         
         LEFT_MOTOR.setSpeed(SPEED);
         RIGHT_MOTOR.setSpeed(SPEED);
         LEFT_MOTOR.forward();
         RIGHT_MOTOR.forward();
       
      /*   try {
          wait(100);
        } catch (InterruptedException e) {
          // TODO Auto-generated catch block
          e.printStackTrace();
        }*/
        }
        else if (odo.getXYT()[0] > project.zone_LL_x*TILE) { //robot reached rLimit. technically should not happen since the robot should not have to go through whole search zone without finding a can
          LEFT_MOTOR.stop(true);
          RIGHT_MOTOR.stop(); 
          motorSweep.stopSensor(); //motor stops sweeping
          //TODO: SHOULD WE END THE MOTORSWEEP THREAD SINCE WE DONT NEED IT ANYMORE?
          return; //ends search
        }
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
      double[] arr = new double[5]; // store readings
      for (int i = 0; i < 5; i++) { // take 5 readingss
        usDistance.fetchSample(usData, 0); // store reading in buffer
        arr[i] = usData[0] * 100.0; // signal amplification
      }
      Arrays.sort(arr); // sort readingss
      return arr[2]; // take median value
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


    


     
