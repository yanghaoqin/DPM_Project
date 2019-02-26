package sshDataReader;

import java.io.FileNotFoundException;
import java.io.PrintWriter;
import java.io.UnsupportedEncodingException;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

public class sshDR {
  
  public static void main(String args[]) throws Exception {
    
    // write out file
    PrintWriter writer = new PrintWriter("data.csv", "UTF-8");
    
    // sensor
    EV3ColorSensor sensor = new EV3ColorSensor(SensorPort.S1);
    SensorModes lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S4"));
    SampleProvider lightColor = ((EV3ColorSensor) lightSensor).getRGBMode();
    float[] csData = new float[3];
    
    for(int i = 0; i < 100; i++) {
      
    }
    
    
  }

}
