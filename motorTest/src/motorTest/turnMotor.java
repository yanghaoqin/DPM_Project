package motorTest;

import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * Class for testing motor turning accuracy. Turns the motor degrees specified by user.
 * 
 * @author Raymond H. Yang
 * @Date: 2019/03/04
 */
public class turnMotor {

  // -----------------------------------------------------------------------------
  // Class Variables
  // -----------------------------------------------------------------------------

  /**
   * Motor instance, specify port motor is connected to. This is the motor to be tested!
   */
  private static final EV3LargeRegulatedMotor L_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("A"));

  /**
   * Motor instance, specify port motor is connected to. This is a support motor
   */
  private static final EV3LargeRegulatedMotor R_MOTOR =
      new EV3LargeRegulatedMotor(LocalEV3.get().getPort("D"));

  /**
   * The amount of turning in degrees
   */
  private static final int DEG = 360;

  /**
   * The acceleration of the motor
   */
  private static final int ACCELERATION = 6000;

  /**
   * The speed of the motor
   */
  private static final int SPEED = 250;

  // -----------------------------------------------------------------------------
  // Main Method
  // -----------------------------------------------------------------------------

  /**
   * Main method
   * 
   * @param args - None
   */
  public static void main(String args[]) {

    // set acceleration and speed
    L_MOTOR.setAcceleration(ACCELERATION);
    L_MOTOR.setSpeed(SPEED);

    // reset count of degrees rotated
    L_MOTOR.resetTachoCount();

    // record current tacho count
    int initialCount = L_MOTOR.getTachoCount();

    // signal status
    System.out.println("Standby...");

    // rotate the motor
    L_MOTOR.rotate(DEG, true);
    R_MOTOR.rotate(DEG, false);
    L_MOTOR.stop(true);
    R_MOTOR.stop(false);

    // record current tacho count
    int finalCount = L_MOTOR.getTachoCount();

    // signal completion
    System.out.println("Complete: " + DEG + " degrees");

    // record rotation
    System.out.println("Recorded Rotation: " + (finalCount - initialCount) + " degrees");

  }
}
