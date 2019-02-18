package ca.mcgill.ecse211.lab5;

public class Search extends Thread{

  private static final int blue = 0; //value of blue colour
  private static final int green = 0; //value of green colour
  private static final int yellow = 0; //value of yellow colour
  private static final int red = 0; //value of red colour
  private static final int TR = 0; //colour of target can: must be changed during demo
  private static int threshold; //colour threshold to identify correct can
  
  public Search() {
    //constructor i will build later
  }
  
  public void run() {
    switch(TR) {
      case 1: threshold = blue; //TR = 1 --> we are looking for blue can
        break;
      case 2: threshold = green; //TR = 2 --> we are looking for a green can
        break;
      case 3: threshold = yellow; //TR = 3 --> we are looking for a yellow can
        break;
      case 4: threshold = red; //TR = 4 --> we are looking for a red can
    }
  }
}
