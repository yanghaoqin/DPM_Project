package ca.mcgill.ecse211.lab5;

import lejos.hardware.Sound;
import lejos.robotics.SampleProvider;

public class CanCalibrator {

  private double std;
  private float[] lightData;
  private SampleProvider lightColor;
  private float filterSum;
  private final int RED = 0;
  private final int GREEN = 1;
  private final int BLUE = 2;
  private final int SCAN_TIME = 100;
  private final int SCAN_LENGTH = 100;

  public CanCalibrator(SampleProvider lightColor, float[] lightData) {
    this.lightData = lightData;
    this.lightColor = lightColor;
    this.std = 0;
  }


  private boolean Calibrate(double[] target, boolean stop) {
    double[] red_array = new double[SCAN_LENGTH];
    double[] green_array = new double[SCAN_LENGTH];
    double[] blue_array = new double[SCAN_LENGTH];
    double[] mean = new double[3];
    double[] standard_deviation = new double[3]; 
    /*
     * while(!stop) {
     * 
     * }
     */
    for (int i = 0; i < SCAN_TIME; i++) {
      red_array[i] = initialReading(RED);
      green_array[i] = initialReading(GREEN);
      blue_array[i] = initialReading(BLUE);
    }

    mean[0] = Find_Mean(red_array);
    mean[1] = Find_Mean(green_array);
    mean[2] = Find_Mean(blue_array);
    
    mean = Mean_Normalizer(mean[0], mean[1], mean[2]);
    
    standard_deviation[0] = Find_Standard_Deviation(red_array, mean[0]);
    standard_deviation[1] = Find_Standard_Deviation(green_array, mean[1]);
    standard_deviation[2] = Find_Standard_Deviation(blue_array, mean[2]);
    
    if(Compare_Standard_Deviation(target[0], standard_deviation[0], mean[0]) && 
        Compare_Standard_Deviation(target[1], standard_deviation[1], mean[1]) &&
        Compare_Standard_Deviation(target[2], standard_deviation[2], mean[2])) {
      Sound.beep();
      return true;
    }
    Sound.beep();
    Sound.beep();
    return false;
  }

  private double Find_Mean(double[] color_array) {
    double mean = 0;
    for (int i = 0; i < color_array.length - 1; i++) {
      mean = mean + color_array[i];
    }
    mean = mean / color_array.length;
    return mean;
  }

  private double Find_Standard_Deviation(double[] color_array, double mean) {
    double stdr_dev = 0;
    for (int i = 0; i < color_array.length - 1; i++) {
      color_array[i] = Math.sqrt(Math.pow(color_array[i] - mean, 2));
    }
    stdr_dev = Math.sqrt(Find_Mean(color_array));
    return stdr_dev;
  }

  private double[] Mean_Normalizer(double red, double green, double blue) {
    double[] rgb = new double[3];
    double euclidean = (Math.sqrt(red * red + green * green + blue * blue));
    rgb[0] = red / euclidean;
    rgb[1] = green / euclidean;
    rgb[2] = blue / euclidean;
    return rgb;
  }

  private boolean Compare_Standard_Deviation(double target, double strd_dev, double mean) {
    if(Math.abs(mean - target) < 2 * strd_dev) {
      return true;
    }
    else {
      return false;
    }
  }
  
  /**
   * initial readings taken (100 times) and the average is used to distinguish between the wooden
   * board and the black line. Each reading is amplified to enhance the sensitivity of the sensor
   * 
   * @return
   */
  private double initialReading(int color_id) {
    for (int i = 0; i < 100; i++) {
      // acquires sample data
      lightColor.fetchSample(lightData, color_id);
      // amplifies and sums the sample data
      std += lightData[color_id] * 100.0;
    }
    // take average of the standard
    return std /= 100.0;
  }
}
