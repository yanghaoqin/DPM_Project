package sshDataReader;

import java.io.PrintWriter;
import java.io.FileWriter;
import lejos.hardware.Button;
import lejos.hardware.Sound;
import lejos.hardware.ev3.LocalEV3;
import lejos.hardware.sensor.*;
import lejos.robotics.SampleProvider;

/**
 * A class to test the light sensors, record data, and output data into csv file. Program is able to
 * read multiple sets of data with multiple readings in each set. Set {@code i} and {@code j} for
 * information.
 * 
 * @author Raymond H. Yang
 * @version 1.0
 * @date 2019/03/12
 */
public class NormTestSSH {

  /**
   * Number of sets of data to read
   */
  private static final int SETS = 4;

  /**
   * Number of readings to read for each set
   */
  private static final int READINGS = 30;

  /**
   * main method
   * 
   * @param args - None
   * @throws Exception - None
   */
  public static void main(String args[]) throws Exception {

    // sensor initialization
    EV3ColorSensor sensor = new EV3ColorSensor(LocalEV3.get().getPort("S1"));
    SampleProvider lightColor = (sensor).getRGBMode();
    float[][] csData = new float[30][3];

    // number of sets of readings
    for (int i = 1; i <= SETS; i++) {

      // write out to file with auto-incrementing file name
      FileWriter fw = new FileWriter("Data" + i + ".csv");
      PrintWriter writer = new PrintWriter(fw);

      // signal
      System.out.println("Ready for " + i);

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
        for (int j = 0; j < READINGS; j++) {

          lightColor.fetchSample(csData[j], 0);
          // System.out.print("R: " + csData[i][0] + " G: " + csData[i][1] + " B: " + csData[i][2] +
          // "\n");

          // write values in specific format
          writer.print(csData[j][0]);
          writer.print(",");

          writer.print(csData[j][1]);
          writer.print(",");

          writer.println(csData[j][2]);

        }

        // indicate termination
        Sound.beep();

      } finally {
        fw.close();
        writer.close();
      }

    }

    sensor.close();

    // exit system
    System.exit(0);

  }

}
