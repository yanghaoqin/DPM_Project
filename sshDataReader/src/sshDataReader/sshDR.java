package sshDataReader;

import java.io.PrintWriter;
import java.io.FileWriter;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

/**
 * A class to test the light sensors, record data, and output data into csv file. A button press is
 * needed for each set of readings.
 * 
 * @author Raymond H. Yang
 * @version 1.0
 */
public class sshDR {

  /**
   * main method
   * 
   * @param args - None
   * @throws Exception - None
   */
  public static void main(String args[]) throws Exception {

    // sensor initialization
    SensorModes lightSensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    SampleProvider lightColor = ((EV3ColorSensor) lightSensor).getRGBMode();
    float[][] csData = new float[100][3];

    // number of sets of readings
    for (int j = 1; j < 6; j++) {

      // write out to file with auto-incrementing file name
      FileWriter fw = new FileWriter("BlueData" + j + ".csv");
      PrintWriter writer = new PrintWriter(fw);

      // signal
      System.out.println("Ready for " + j);

      // press button to start
      Button.waitForAnyPress();

      // header
      writer.print("R");
      writer.print(",");
      writer.print("G");
      writer.print(",");
      writer.println("B");

      try {

        // read n times and save data
        for (int i = 0; i < 10; i++) {

          lightColor.fetchSample(csData[i], 0);
          // System.out.print("R: " + csData[i][0] + " G: " + csData[i][1] + " B: " + csData[i][2] +
          // "\n");

          // write values in specific format
          writer.print(csData[i][0]);
          writer.print(",");

          writer.print(csData[i][1]);
          writer.print(",");

          writer.println(csData[i][2]);

        }

        // indicate termination
        Sound.beep();

      } finally {
        fw.close();
        writer.close();
      }

    }

    // exit system
    System.exit(0);

  }

}
