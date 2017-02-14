* IMPORTANT NOTES:
 * 
 * 1) WHERE FILES/FOLDERS SHOULD BE PLACED IN ORDER TO MAKE 
 *    EVERYTHING COMPILE AND WORK CORRECTLY (on MIRTO's SD
 *    CARD):
 *    (i) MobileRobot.java in /csd2222
 *    (ii) RobotMission.java in /csd2222
 *    (iii) RobotTesting.java in /csd2222
 *    (iv) The following files and folders should also be placed
 *         in /csd2222: java-asip.jar, jssc, libs, META-INF and 
 *         uk.
 *    (v) The modified version of the file JMirtoRobot.java (see
 *        below) should be placed in 
 *        /csd2222/uk/ac/mdx/cs/asip
 * 
 *    Source: https://github.com/fraimondi/java-asip
 * 
 * 2) The code in JMirtoRobot.java has been modified by Claudio S. 
 * De Mutiis (claudio.de_mutiis@kcl.ac.uk) in August 2015. 
 * Claudio S. De Mutiis added the method public void resetCount(),
 * which resets both of the encoders' counts.
 * 
 *  public void resetCount() {
 *    e0.resetCount();
 *    e1.resetCount();
 *  }
 * 
 * TRYING TO COMPILE THE CLASSES MobileRobot.java, RobotMission.java 
 * and RobotTesting.java WITH THE OLD VERSION OF JMirtoRobot.java
 * WILL RESULT IN A COMPILATION ERROR !!!
 * 
 * The modified version of JMirtoRobot.java is needed to make 
 * everything compile and run correctly !!
 * 
 * ADDITIONAL CREDITS:
 * - The class JMirtoRobot.java was developed by a team led by Dr 
 *   Franco Raimondi (F.Raimondi@mdx.ac.uk) at Middlesex University 
 *   London. When using the functionalities of the class 
 *   JMirtoRobot.java, in several occasions, I used or slightly 
 *   modified snippets of code taken from the file JMirtoRobot.java. 
 *   I copied and pasted the main method for testing the class 
 *   JMirtoRobot below.
 * - In order to be able to work, the classes MobileRobot.java,
 *   RobotMission.java and RobotTesting.java make 
 *   use of methods and classes developed by a team led by Dr
 *   Franco Raimondi at Middlesex University (see the folders lib 
 *   and src and the files build.xml, README.md located in the 
 *   folder java-asip-master).
 *   Source: https://github.com/fraimondi/java-asip  
 */

/*  Useful Math Functions:
 * 1) Math.toDegrees(...) --> converts an angle from rad to deg
 * 2) Math.toRadians(...) --> converts an angle from deg to rad
 */

 * The main method for testing the class JMirtoRobot.java, was written by a team led 
 * by Dr Franco Raimondi (F.Raimondi@mdx.ac.uk) at Middlesex University London 
 *
 * 
 *  // A main method for testing
 *  public static void main(String[] args) {
 * 
 *  // JMirtoRobot robot = new JMirtoRobot("/dev/tty.usbserial-A903VH1D");
 *  JMirtoRobot robot = new JMirtoRobot("/dev/ttyAMA0");
 *
 * 
 *  try {
 *  Thread.sleep(500);
 *  robot.setup();
 *  Thread.sleep(500); 
 *  while (true) {
 *  System.out.println("IR: "+robot.getIR(0) + ","+robot.getIR(1)+",
 *                     "+robot.getIR(2));
 *  System.out.println("Encoders: "+robot.getCount(0) + ","+robot.getCount(1));
 *  System.out.println("Bumpers: "+robot.isPressed(0) + ","+robot.isPressed(1));
 *  System.out.println("Setting motors to 50,50");
 *  robot.setMotors(100, 0);
 *  Thread.sleep(1500);
 *  System.out.println("Stopping motors");
 *  robot.stopMotors();
 *  Thread.sleep(500);
 *  System.out.println("Setting motors to 100,100");
 *  robot.setMotors(0,-250);
 *  Thread.sleep(1500);
 *  System.out.println("Stopping motors");
 *  robot.stopMotors();
 *  Thread.sleep(500);
 *  }
 *  /*   System.out.println("Setting motors to 50,50");
 *       robot.setMotors(50, 50);
 *       Thread.sleep(3000);
 *       System.out.println("Stopping motors");
 *       robot.stopMotors();
 *       Thread.sleep(500);
 *       System.out.println("Setting motors to 80,-80");
 *       robot.setMotors(80, -80);
 *       Thread.sleep(3000);
 *       System.out.println("Stopping motors");
 *       robot.stopMotors();
 *       Thread.sleep(3000);
 *       System.out.println("Setting motors to -100,100");
 *       robot.setMotors(-100, 100);
 *       Thread.sleep(3000);
 *       System.out.println("Stopping motors");
 *       robot.stopMotors();
 *       System.out.println("All done, see you soon!");
 *  */
 /*
  * } catch (InterruptedException e) {
  * e.printStackTrace();
  * }
  *
  * }
 * 
 */