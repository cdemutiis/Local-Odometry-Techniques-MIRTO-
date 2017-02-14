/* File: RobotTesting.java
 * Date: August 2015
 * King's College London -- Dept. of Informatics -- MSc in Robotics
 * Author: Claudio S. De Mutiis (claudio.de_mutiis@kcl.ac.uk)
 * Purpose: Test the basic rotational and translational capabilities 
 *          of the MIRTO robot, developed by a team led by Dr Franco 
 *          Raimondi (F.Raimondi@mdx.ac.uk) at Middlesex University 
 *          London 
 * 
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
 *   London. 
 * - In order to be able to work, the class RobotTesting.java makes 
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
/********************************************************************
/* VERY USEFUL FUNCTIONS OF MobileRobot.java (see the comments 
 * of MobileRobot.java file to learn about other available functions) 
 * ******************************************************************
 * public void translate(double dist) --> Make the mobile robot 
 *                      translate by "dist" cm
 * public void rotate(double change) --> Make the mobile robot rotate 
 *                      by "change" degrees
 * public void stop() --> Make the mobile robot stop
 * public void reset(Point goal_new) --> Reset the goal point in cm 
 * public void reset(Point goal_new, double start_orientation_new) 
 *                      --> Reset the goal point in cm and orientation 
 *                      in degrees
 * public void goToGoal() --> Make the mobile robot move to its goal 
 *                      point 
 * public void goToGoal(Point goal_new) --> Make the mobile robot move 
 *                      to the new goal point 
 * ********************************************************************
 * 
 */

public class RobotTesting {
     public static void main(String[] args) {
         // create a start point at the origin (0,0) --> current 
         // location of the robot 
         Point start = new Point();
         // create a goal point at (50, 50)
         Point goal = new Point(50,50); 
         // specify the current orientation of the robot
         double orientation = 180;  
         // create the robot object 
         MobileRobot robot = new MobileRobot(start, goal, orientation); 
         robot.translate(20);
         robot.translate(-20);
         robot.translate(20);
         robot.rotate(-180);
         robot.rotate(90);
     }
}