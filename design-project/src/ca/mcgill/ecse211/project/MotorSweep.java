package ca.mcgill.ecse211.project;

//import lejos.hardware.motor.EV3MediumRegulatedMotor;
import lejos.hardware.motor.NXTRegulatedMotor;

/**
 * This class controls the sweeps of the motor that is attached to the ultrasonic sensor
 *  and the initialization of the sensor to its starting position. It is used during the searching phase of our project
 *  
 * @author Antoine Wang
 * @author Tudor Gurau
 * @author Erica De Petrillo
 *
 */
public class MotorSweep extends Thread {
    private NXTRegulatedMotor sensorMotor;
    private static boolean isRotating = true;
    private static boolean upright = false;
    private static final int SWEEP_ANGLE = 70;
    public float straightTacho; //used to keep the tacho count when the sensor is straight
   
    /**
     * Class constructor
     * @param sensormotor2 the Medium Motor of the robot, attached to the ultrasonic sensor
     */
    public MotorSweep(NXTRegulatedMotor sensormotor2){
        this.sensorMotor = sensormotor2;
    }
    
    /**
     * This method sets the flag controlling the sweeping to false, so the sweeping stops
     */
    public void stopSensor(){
        isRotating = false; 
    }
    
    /**
     * This method sets the flag controlling the sweeping to true, so the sweeping starts
     */
    public void startSensor(){
        isRotating = true;
    }
    
    /**
     * This method defines whether the sensor should be reset upright
     */
    public void setPos(){
        upright = true;
    }
    
    /**
     * This method defines the movements of the sweeping motor, and
     * updates the tachometer count every time the sensor is straight.
     */
        
    public void run(){
        sensorMotor.setSpeed(100);
        while(true){
            while(isRotating){     // this while loop controls whether the sensor is sweeping
                sensorMotor.rotate(SWEEP_ANGLE);        
                sensorMotor.rotate(-SWEEP_ANGLE);
                straightTacho = sensorMotor.getTachoCount(); //get tacho count when sensor back straight
                sensorMotor.rotate(-SWEEP_ANGLE);
                sensorMotor.rotate(SWEEP_ANGLE);
            }
            while(upright){       // this while loop set the sensor to face front only once
                
                sensorMotor.rotateTo(0, false);
                straightTacho = sensorMotor.getTachoCount();
                upright = false;
                
                }
        }
        
    }
    public void ReinitiateMotor(NXTRegulatedMotor SensorMotor, int InitialTacho) {
      
    }
}