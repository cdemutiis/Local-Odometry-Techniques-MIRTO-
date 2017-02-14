/* File: MobileRobot.java
 * Date: June 2015
 * King's College London -- Dept. of Informatics -- MSc in Robotics
 * Author: Claudio S. De Mutiis (claudio.de_mutiis@kcl.ac.uk)
 * Purpose: Provide the MIRTO robot, developed by a team led by Dr 
 * Franco Raimondi (F.Raimondi@mdx.ac.uk) at Middlesex University 
 * London, with high-level odometrical functionalities.
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
 *   London. When using the functionalities of the class 
 *   JMirtoRobot.java, in several occasions, I used or slightly 
 *   modified snippets of code taken from the file JMirtoRobot.java. 
 *   I copied and pasted the main method for testing the class 
 *   JMirtoRobot below in the comments (after the description for 
 *   the class Point).
 * - In order to be able to work, the class MobileRobot.java makes 
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

/*************************** MobileRobot ***************************
 * public MobileRobot() --> Constructor with no arguments
 * public MobileRobot(Point start_new) --> Constructor with the 
 *                   start point coordinates in cm
 * public MobileRobot(Point start_new, double orientation_new) --> 
 *                   Constructor with start point coordinates in cm 
 *                   and orientation in degrees 
 * public MobileRobot(Point start_new, Point goal_new) --> 
 *                   Constructor with start and goal points 
 *                   coordinates in cm
 * public MobileRobot(Point start_new, Point goal_new, double 
 *                   orientation_new) --> Constructor with start and 
 *                   goal coordinates in cm and orientation in degrees 
 * public Point getLocation() --> Get the current location of the
 *                   mobile robot (x,y) -- cm
 * public void setStart(Point start_new) --> Set the start point (x,y) 
 *                   -- cm
 * public Point getStart() --> Get the start point (x,y) -- cm
 * public void setGoal(Point goal_new) --> Set the goal point (x,y) 
 *                   -- cm
 * public Point getGoal() --> Get the goal point (x,y) -- cm
 * public void setOrientation(double orientation_new) --> Set the 
 *                   orientation of the robot -- degrees
 * public double getOrientation() --> Get the current orientation of 
 *                   the robot with respect to the positive x-axis -- 
 *                   degrees
 * public double getStartOrientation() --> Get the initial orientation 
 *                   of the robot with respect to the positive x-axis 
 *                   -- degrees
 * public void setSpeed(double speed_left, double speed_right) --> Set 
 *                   the speeds of the right and left wheel -- cm/s
 * public Speed getSpeed() --> Get the speeds of the right and left 
 *                   wheel -- cm/s
 * public double getLinSpeed() --> Get the linear speed of the mobile 
 *                   robot -- cm/s
 * public double getAngSpeed() --> Get the angular speed of the mobile 
 *                   robot -- radians/sec
 * public void translate(double dist) --> Make the mobile robot 
 *                   translate by "dist" cm
 * public void rotate(double change) --> Make the mobile robot rotate 
 *                   by "change" degrees
 * public void stop() --> Make the mobile robot stop
 * public void reset(Point goal_new) --> Reset the goal point coordinates 
 *                   in cm
 * public void reset(Point goal_new, double start_orientation_new) --> 
 *                   Reset the goal point in cm and orientation of the 
 *                   robot in degrees
 * public void goToGoal() --> Make the mobile robot move to its goal point 
 * public void goToGoal(Point goal_new) --> Make the mobile robot move to 
 *                   the new Goal Point 
 * public void motion_to_goal() --> A variant of the Motion to Goal Behavior 
 *                   that recalculates the slope between the current location 
 *                   and the goal point at the beginning of every iteration 
 *                   of the while loop. If an obstacle gets detected, an "hit" 
 *                   point is saved on a list and the mobile robot starts its 
 *                   Wall-Following Behavior. If the goal has been reached, 
 *                   nothing gets done by the algorithm.
 * public void wall_following() --> A "greedy" version of the Wall-Following 
 *                   Behavior that switches to Motion to Goal Behavior either 
 *                   if the method public boolean goal_line_hit(double 
 *                   init_slope) returns true or if more than 5 iterations of 
 *                   the Wall-Following Behavior have been executed.If the 
 *                   goal has been reached, nothing gets done by the 
 *                   algorithm.
 * public boolean obstacle_detected() --> Returns true if an obstacle has 
 *                   been detected by the mobile robot's bumpers
 * public void avoid_obstacle(char ch) --> Determines which bumper detected 
 *                   the obstacle and makes the robot act appropriately ('l' 
 *                   and 'r' for counterclockwise and clockwise rotation,
 *                   respectively). This code has been written in a way to a 
 *                   allow further development for the programmer who wants 
 *                   the robot to perform different actions depending on which 
 *                   bumpers' sensors are activated.
 * public boolean goal_line_hit(double init_slope) --> Returns true if the 
 *                   mobile robot hits the goal slope it stored at the 
 *                   beginning of the Wall-Following Behavior, i.e. init_slope, 
 *                   at a "hit" point that has not been visited yet and the 
 *                   function goal_line_hit(double init_slope) has been called 
 *                   at least 2 times. It also returns true if init_slope is 
 *                   greater than 5, if the absolute difference between the 
 *                   x-coordinates of the current location and goal point is 
 *                   less than 5 and if the function 
 *                   goal_line_hit(double init_slope) has been called at least 
 *                   2 times.It returns false otherwise. 
 * public boolean goal_reached() --> Returns true if the robot is within 7 cm of 
 *                   its current goal point
 * public double getGoalSlope() --> Get the slope of the line connecting the 
 *                   start point and the goal point  
 * public double getLocationSlope() --> Get the slope of the line conecting the 
 *                   goal point and the current location of the robot
 * public boolean hp_visited() --> Returns true if the mobile robot crosses one 
 *                   of the "hit" points previously visited 
 * public double get_enc_distance() --> Get the "encoder" distance traveled by 
 *                   the mobile robot
 * public double get_cum_distance() --> Get the cumulative distance traveled by 
 *                   the mobile robot
 * public void print_location(String current) --> Print the current location of 
 *                   the robot 
 * *****************************************************************************
 * 
 * *************** Speed (part of the class MobileRobot.java) *******************
 * public Speed() --> Constructor for zero Speed:(0,0)
 * public Speed(double vr_new, double vl_new) --> Constructor for Speed:
 *                  (speed_right_wheel, speed_left_wheel)
 * public void set_vr(double vr_new) --> Set the speed of the right wheel
 * public double get_vr() --> Get the speed of the right wheel
 * public void set_vl(double vl_new) --> Set the speed of the left wheel
 * public double get_vl() --> Get the speed of the left wheel
 * *******************************************************************************
 * 
 * ************************************ Point **********************************
 * public Point() --> Constructor for the origin point:(0,0)
 * public Point(double x_new, double y_new) --> Constructor for a Point:(x,y)
 * public void setX(double x_new) --> Set the x-coordinate of the point 
 * public double getX() --> Get the x-coordinate of the point
 * public void setY(double y_new) --> Set the y-coordinate of the point
 * public double getY() --> Get the y-coordinate of the point
 * public double distanceTo(Point point) --> Get the distance of THIS point to 
 *                   another point 
 * ******************************************************************************
 * 
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

import uk.ac.mdx.cs.asip.JMirtoRobot;
import uk.ac.mdx.cs.asip.services.BumpService;
import uk.ac.mdx.cs.asip.services.EncoderService;
import uk.ac.mdx.cs.asip.services.IRService;
import uk.ac.mdx.cs.asip.services.MotorService;
import java.util.*;

public class MobileRobot {
    // Distance between the two robot's wheels in cm
    private static final double l = 11.5;  
    // Forward Motor-Speed Conversion Factor for laminate flooring 
    // (left wheel)
    private static final double speed_factor_left_f = 8.1; 
    // Forward Motor-Speed Conversion Factor for laminate flooring 
    // (right wheel)
    private static final double speed_factor_right_f = 8.7; 
    // Backward Motor-Speed Conversion Factor for laminate flooring 
    // (left wheel)
    private static final double speed_factor_left_b = 8.3; 
    // Backward Motor-Speed Conversion Factor for laminate flooring 
    // (right wheel)
    private static final double speed_factor_right_b = 8.9; 
    // Encoder count for a full 360 degrees rotation 
    private static final double enc_full = 125;  
    // Start Point (x,y) -- cm 
    private Point start = new Point(); 
    // Goal Point (x,y) -- cm
    private Point goal = new Point(); 
    // Current location of the mobile robot (x,y) -- cm 
    private Point location = new Point(); 
    // Speed (right wheel, left wheel) -- cm/s
    private Speed speed = new Speed(); 
    // Initial orientation of the robot with respect to the x-axis 
    // -- degrees
    private double start_orientation; 
    // Current orientation of the robot with respect to the x-axis 
    // -- degrees
    private double orientation; 
    // Linear Speed of the robot -- cm/s 
    private double linear_speed; 
    // Angular Speed of the robot -- radians/sec
    private double angSpeed;
    // Robot (control motors and sensors)
    public JMirtoRobot robot = new JMirtoRobot("/dev/ttyAMA0"); 
    // Set of the 2D Cartesian "hit" points visited by the mobile 
    // robot 
    private Set<Point> visited_points = new HashSet<Point>();  
    // Number of iterations of the wall-following behaviour
    private int count_wall;
    // Cumulative distance traveled by the mobile robot
    private double cum_distance; 
    // Slope from start to goal point
    private double goal_slope; 
    // Number of times the function goal_line_hit() gets called
    private int count_glh = 0; 
    // Counter for the number of translations when the robot 
    // gets stuck on an obstacle
    private int count_trans = 0; 
    // Counter for the number of times the function 
    // goal_reached() gets called and returns true
    private int count_goal_reached = 0; 
    
    
    // Constructor with no arguments
    public MobileRobot() {
        try {
        location.setX(0);
        location.setY(0);
        goal.setX(0);
        goal.setY(150);
        robot.setup();
        robot.resetCount();
        Thread.sleep(500); 
        stop();
        start.setX(0);
        start.setY(0);
        start_orientation = 90;
        orientation = 90;
        linear_speed = 0;
        angSpeed = 0;
        cum_distance = 0;
        visited_points = new HashSet<Point>();
        print_location("initial");
        System.out.println();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // Constructor with the start point coordinates in cm    
    public MobileRobot(Point start_new) {
        try {
        location.setX(start_new.getX());
        location.setY(start_new.getY());
        goal.setX(0);
        goal.setY(150);
        robot.setup();
        robot.resetCount();
        Thread.sleep(500); 
        stop();
        start.setX(start_new.getX());
        start.setY(start_new.getY());
        start_orientation = 90;
        orientation = 90;
        linear_speed = 0;
        angSpeed = 0;
        cum_distance = 0;
        visited_points = new HashSet<Point>();
        print_location("initial");
        System.out.println();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
   
    
    // Constructor with start point coordinates in cm and orientation 
    // in degrees   
    public MobileRobot(Point start_new, double start_orientation_new) {
        try {
        location.setX(start_new.getX());
        location.setY(start_new.getY());
        goal.setX(0);
        goal.setY(150);
        robot.setup();
        robot.resetCount();
        Thread.sleep(500); 
        stop();
        start.setX(start_new.getX());
        start.setY(start_new.getY());
        start_orientation = start_orientation_new;
        orientation = start_orientation_new;
        linear_speed = 0;
        angSpeed = 0;
        cum_distance = 0;
        visited_points = new HashSet<Point>();
        print_location("initial");
        System.out.println();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // Constructor with start and goal coordinates in cm 
    public MobileRobot(Point start_new, Point goal_new) {
        try {
        location.setX(start_new.getX());
        location.setY(start_new.getY());
        goal.setX(goal_new.getX());
        goal.setY(goal_new.getY());
        robot.setup();
        robot.resetCount();
        Thread.sleep(500); 
        stop();
        start.setX(start_new.getX());
        start.setY(start_new.getY());
        start_orientation = 90;
        orientation = 90;
        linear_speed = 0;
        angSpeed = 0;
        cum_distance = 0;
        visited_points = new HashSet<Point>();
        print_location("initial");
        System.out.println();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // Constructor with start and goal coordinates in cm
    // and orientation in degrees
    public MobileRobot(Point start_new, Point goal_new, double start_orientation_new) {
        try {
        location.setX(start_new.getX());
        location.setY(start_new.getY());
        goal.setX(goal_new.getX());
        goal.setY(goal_new.getY());
        robot.setup();
        robot.resetCount();
        Thread.sleep(500); 
        stop();
        start.setX(start_new.getX());
        start.setY(start_new.getY());
        start_orientation = start_orientation_new;
        orientation = start_orientation_new;
        linear_speed = 0;
        angSpeed = 0;
        cum_distance = 0;
        visited_points = new HashSet<Point>();
        print_location("initial");
        System.out.println();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // Get the current location of the mobile robot (x,y) -- cm
    public Point getLocation() {
        return location;
    }
    
    // Set the start point (x,y) -- cm
    public void setStart(Point start_new) {
        start.setX(start_new.getX());
        start.setY(start_new.getY());
        location.setX(start_new.getX());
        location.setY(start_new.getY());
    }
    
    // Get the start point (x,y) -- cm
    public Point getStart() {
        return start;
    }
    
    // Set the goal point (x,y) -- cm
    public void setGoal(Point goal_new) {
        goal.setX(goal_new.getX());
        goal.setY(goal_new.getY());
    }
    
    // Get the goal point (x,y) -- cm
    public Point getGoal() {
        return goal;
    }
    
    // Set the orientation of the robot -- degrees
    public void setOrientation(double orientation_new) {
        double rotation;
        if (Math.abs(orientation_new - orientation) > 180) {
            if (orientation_new > orientation) {
                rotation = -1*(360 - orientation_new + orientation); 
            }else {
                rotation = 360 - orientation + orientation_new;
            }
        }else {
            rotation = orientation_new - orientation;  
        }
        rotate(rotation);
    }
    
    // Get the current orientation of the robot with respect to the 
    // positive x-axis -- degrees
    public double getOrientation() {
        return orientation;
    }
    
    // Get the initial orientation of the robot with respect to the 
    // positive x-axis -- degrees
    public double getStartOrientation() {
        return start_orientation;
    }
    
    // Set the speeds of the right and left wheel -- cm/s
    public void setSpeed(double speed_left, double speed_right) {
        if (!goal_reached()) {
            System.out.println("Setting motors to (" + speed_left + "," + speed_right + ")");
            System.out.println();
        }
        speed.set_vr(speed_right);
        speed.set_vl(speed_left);
        Long speed_l = Math.round(speed_left);
        int speed_left_new = Integer.valueOf(speed_l.intValue());
        Long speed_r = Math.round(speed_right);
        int speed_right_new = Integer.valueOf(speed_r.intValue());  
        if (!((Math.abs(speed_left)) > 50 || (Math.abs(speed_right) > 50))) {
            if (speed_left >= 0) {
                speed_l = Math.round(speed_left*speed_factor_left_f);
            }else {
                speed_l = Math.round(speed_left*speed_factor_left_b);
            }
            speed_left_new = Integer.valueOf(speed_l.intValue());
            if (speed_right < 0) {   
                speed_r = Math.round(speed_right*speed_factor_right_f); 
            }else {
                speed_r = Math.round(speed_right*speed_factor_right_b);
            }
            speed_right_new = Integer.valueOf(speed_r.intValue()); 
        }
        robot.setMotors(speed_left_new, speed_right_new);
    }
   
    // Get the speeds of the right and left wheel -- cm/s
    public Speed getSpeed() {
        return speed;
    }
    
    // Get the linear speed of the mobile robot -- cm/s
    public double getLinSpeed() {
        return (speed.get_vr() + speed.get_vl())/2; 
    }
    
    // Get the angular speed of the mobile robot -- radians/sec
    public double getAngSpeed() {
        return 2*(speed.get_vr() - speed.get_vl())/l;
    }
    
    // Make the mobile robot translate by "dist" cm. If the robot has 
    // hit an obstacle not detected by the bumpers' sensors, perform an 
    // evasion manoeuvre.
    public void translate(double dist) {
        try {
            if (!goal_reached()) {
                if (getLocation().distanceTo(getGoal()) < 50 && dist > 0) { 
                    dist = 20;
                    if (getLocation().distanceTo(getGoal()) < 30) {
                        dist = 10;
                        if (getLocation().distanceTo(getGoal()) < 20) {
                            dist = 5;
                            if (getLocation().distanceTo(getGoal()) < 10) {
                                dist = 2;
                                if (getLocation().distanceTo(getGoal()) < 6) {
                                    dist = 1;
                                }
                            }
                        }
                    }
                }
                Point before_translation = new Point(location.getX(), location.getY());
                int time = 0;
                if (dist >= 0) {
                    Long time_l = Math.round(1000*Math.abs(dist)/(93/speed_factor_left_f));
                    time = Integer.valueOf(time_l.intValue());
                    setSpeed(93, -100);  
                }else {
                    Long time_l = Math.round(1000*Math.abs(dist)/(93/speed_factor_left_b));
                    time = Integer.valueOf(time_l.intValue());
                    setSpeed(-93, 100);  
                }
                Thread.sleep(time);
                stop();
                dist = get_enc_distance();
                cum_distance = cum_distance + Math.abs(dist); // Use wheels' encoders 
                System.out.print("Translation of " + dist + " cm executed! ");
                System.out.println("(" + time + " ms)");
                location.setX(location.getX() + dist*Math.cos(Math.toRadians(orientation)));    
                location.setY(location.getY() + dist*Math.sin(Math.toRadians(orientation)));
                print_location("new");
                System.out.print("The current location-goal slope is ");
                System.out.println(getLocationSlope());
                System.out.println();
                Point after_translation = new Point(location.getX(), location.getY());
                if (before_translation.distanceTo(after_translation) < 2) {
                    count_trans++; 
                    if (count_trans == 2) {
                        rotate(5);
                        translate(-10);
                        rotate(30);
                        translate(30);
                        count_trans = 0;
                        if (obstacle_detected()) {
                            wall_following();
                        } else {
                            motion_to_goal();
                        }
                    } 
                    if (count_trans > 2) {
                        rotate(-5);
                        translate(-10);
                        rotate(-30);
                        translate(30);
                        count_trans = 0;
                        if (obstacle_detected()) {
                            wall_following();
                        } else {
                            motion_to_goal();
                        }
                    }
                }else {
                    count_trans = 0; 
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // Make the mobile robot rotate by "change" degrees. Try to correct 
    // a systematic error by subtracting 1 degree to the final 
    // orientation of the robot.
    public void rotate(double change) {
        try {
            if (!goal_reached()) {
                Long time_l;
                int time;
                if (change >= 0) {
                    time_l = Math.round(3810*(Math.abs(change)/360));
                    time = Integer.valueOf(time_l.intValue());
                    setSpeed(-95, -100);
                }else {
                    time_l = Math.round(3640*(Math.abs(change)/360)); 
                    time = Integer.valueOf(time_l.intValue());
                    setSpeed(95, 100);
                }
                Thread.sleep(time); 
                stop();
                change = get_enc_rotation();
                System.out.print("Rotation of " + change);
                System.out.println(" degrees executed! (" + time + " ms)");
                orientation = (orientation + change)%360;
                if (orientation < 0) {
                    orientation = orientation + 360;
                }
                orientation = orientation - 1;
                print_location("new");
                System.out.print("The current location-goal slope is ");
                System.out.println(getLocationSlope());
                System.out.println();
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // Make the mobile robot stop
    public void stop() {
        try {
            if (!goal_reached()) {
                System.out.println("Stopping motors");
                System.out.println();
            }
            robot.stopMotors();
            Thread.sleep(500);
            speed.set_vr(0);
            speed.set_vl(0);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // Reset the goal point coordinates in cm 
    public void reset(Point goal_new) {
        robot.resetCount();
        stop();
        start.setX(location.getX());
        start.setY(location.getY());
        goal.setX(goal_new.getX());
        goal.setY(goal_new.getY());
        cum_distance = 0;
        print_location("initial");
        System.out.println();
        visited_points.clear();
    }
    
    // Reset the goal point coordinates in cm and orientation in 
    //degrees
    public void reset(Point goal_new, double start_orientation_new) {
        robot.resetCount();
        stop();
        start.setX(location.getX());
        start.setY(location.getY());
        goal.setX(goal_new.getX());
        goal.setY(goal_new.getY());
        start_orientation = start_orientation_new;
        orientation = start_orientation_new;
        cum_distance = 0;
        print_location("initial");
        System.out.println();
        visited_points.clear();
    }
    
    // Make the mobile robot move to its Goal Point 
    public void goToGoal() {
        try {
            Thread.sleep(700);
            if (!goal_reached()) {
                goal_slope = getLocationSlope();
                System.out.println("The current location-goal slope is " + goal_slope); 
                if (!obstacle_detected()) {
                    motion_to_goal();
                }else {
                    Point temp_location = getLocation();  
                    visited_points.add(temp_location); 
                    wall_following(); 
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // Make the mobile robot move to the new Goal Point 
    public void goToGoal(Point goal_new) {
        try {
            Thread.sleep(700);
            goal.setX(goal_new.getX());
            goal.setY(goal_new.getY());
            Thread.sleep(700);
            if (!goal_reached()) {
                goal_slope = getLocationSlope();
                System.out.println("The current location-goal slope is " + goal_slope); 
                if (!obstacle_detected()) {
                    motion_to_goal();
                }else {
                    Point temp_location = getLocation();  
                    visited_points.add(temp_location); 
                    wall_following(); 
                }
            }
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
    
    // A variant of the Motion to Goal Behavior that recalculates the slope between 
    // the current location and the goal point at the beginning of every iteration 
    // of the while loop. If an obstacle gets detected, an "hit" point is saved on a 
    // list and the mobile robot starts its Wall-Following Behavior. If the goal has 
    // been reached, nothing gets done by the algorithm.
    public void motion_to_goal() {
        if (!goal_reached()) {
            System.out.println();
            System.out.println();
            System.out.println("********** Motion to Goal Behavior ... **********");
            System.out.println("********** Motion to Goal  Behavior ... **********");
            System.out.println("********** Motion to Goal Behavior ... **********");
            System.out.println();
            System.out.println();
            while (!obstacle_detected() && !goal_reached()) {
                goal_slope = getLocationSlope();
                double orientation_new = Math.atan(goal_slope);
                orientation_new = Math.toDegrees(orientation_new);
                double orientation_1 = orientation_new;
                double orientation_2 = orientation_new + 180;
                if (orientation_1 > 0) {
                    if (goal.getY() > location.getY()) {
                        orientation_new = orientation_1;
                    }else {
                        orientation_new = orientation_2;
                    }
                } else if (orientation_1 < 0) {
                    if (goal.getY() > location.getY()) {
                        orientation_new = orientation_2;
                    }else {
                        orientation_new = orientation_1;
                    }
                } else {
                    if (goal.getX() > location.getX()) {
                        orientation_new = orientation_1;
                    }else {
                        orientation_new = orientation_2;
                    }
                }
                if (orientation_new < 0) {
                    orientation_new = 360 + orientation_new;
                }
                setOrientation(orientation_new);
                translate(30); 
            }
            if (!goal_reached()) {
                Point temp_location = getLocation();  
                visited_points.add(temp_location); 
                wall_following();
            }
        }
    }
    
    // A "greedy" version of the Wall-Following Behavior that switches to Motion to 
    // Goal Behavior either if the method public boolean goal_line_hit(double init_slope) 
    // returns true or if more than 5 iterations of the Wall-Following Behavior have 
    // been executed.If the goal has been reached, nothing gets done by the algorithm.
    public void wall_following() { 
        if (!goal_reached()) {
            System.out.println();
            System.out.println();
            System.out.println("********** Wall-Following Behavior ... **********");
            System.out.println("********** Wall-Following Behavior ... **********");
            System.out.println("********** Wall-Following Behavior ... **********");
            System.out.println();
            System.out.println();
            count_wall = 0;
            double init_slope = getLocationSlope();
            while ((!goal_line_hit(init_slope)  && count_wall <= 5) && !goal_reached()) {
                while (obstacle_detected() && !goal_reached()) {
                    // make the robot be approximately parallel to the 
                    // obstacle
                    avoid_obstacle('l');
                    translate(12);
                }
                // rotate 90 degrees clockwise
                if (orientation - 90 < 0) {    
                    setOrientation(orientation - 90 + 360);
                }else {
                    setOrientation(orientation - 90); 
                }
                translate(12);
                count_wall++;
                if (count_wall > 5) {
                    translate(-10);
                    if (!goal_reached()) {
                        System.out.println();
                        System.out.print("****** Leaving Wall-Following Behavior ... ");
                        System.out.println("trying to get to the goal ! ******");
                        System.out.println();
                        motion_to_goal();
                    }   
                }  
            }
            if (!goal_reached()) {
                System.out.println();
                System.out.print("****** Leaving Wall-Following Behavior ... ");
                System.out.println("trying to get to the goal ! ******");
                System.out.println();
                motion_to_goal();
            }
        }
    }
    
    // Returns true if an obstacle has been detected by the mobile 
    // robot's sensors
    public boolean obstacle_detected() { 
        if (robot.isPressed(0) || robot.isPressed(1)) {
            if (!goal_reached()) {
                System.out.println();
                System.out.println("****** An obstacle has been hit !!! ******");
                System.out.println();
            }
            return true;
        }else {
            return false;
        }
    }

    // Determines which bumper detected the obstacle and makes the robot 
    // act appropriately ('l' and 'r' for counterclockwise and clockwise 
    // rotation, respectively). This code has been written in a way to a 
    // allow further development for the programmer who wants the robot 
    // to perform different actions depending on which bumpers' sensors 
    // are activated.
    public void avoid_obstacle(char ch) {
        double angle;
        double trans = -8;
        if (ch == 'l') {
            angle = 90; 
        }else if (ch == 'r') {
            angle = -90;
        }else {
            angle = 180;
        }
        if (obstacle_detected()) {
            if (robot.isPressed(0) && robot.isPressed(1)) {
                translate(trans);
                if (orientation + angle < 0) {    
                    setOrientation(orientation + angle + 360);
                }else {
                    setOrientation((orientation + angle)%360); 
                }
            }else if (robot.isPressed(0) && (!robot.isPressed(1))) {
                translate(trans);
                if (orientation - angle < 0) {    
                    setOrientation(orientation + angle + 360);
                }else {
                    setOrientation((orientation + angle)%360); 
                }
            }else {
                translate(trans); 
                if (orientation - angle < 0) {    
                    setOrientation(orientation + angle + 360);
                }else {
                    setOrientation((orientation + angle)%360); 
                }
            }
        }
    }
   
    // Returns true if the mobile robot hits the goal slope it stored at 
    // the beginning of the Wall-Following Behavior, i.e. init_slope, at 
    // a "hit" point that has not been visited yet and the function 
    // goal_line_hit(double init_slope) has been called at least 2 times. 
    // It also returns true if init_slope is greater than 5, if the 
    // absolute difference between the x-coordinates of the current 
    // location and goal point is less than 5 and if the function 
    // goal_line_hit(double init_slope) has been called at least 2 times.
    // It returns false otherwise.
    public boolean goal_line_hit(double init_slope) {
        double location_slope = getLocationSlope();
        boolean flag = false;
        if (count_glh >= 2) {
            if ((100*Math.abs(init_slope - location_slope)/init_slope < 5) && !hp_visited()) {
                flag = true;
            }
            if ((init_slope > 5) && (Math.abs(location.getX() - goal.getX()) < 5)) {
                flag = true;
            }
        }

        if (flag == true) {
            if (!goal_reached()) {
                System.out.println();
                System.out.println("****** The start-goal line has been hit!! ******");
                System.out.println();
            }
            count_glh = 0;
            return true;   
        }else {
            count_glh++;
            return false;
        }
        
    }

    // Returns true if the robot is within 7 cm of its current goal 
    // point
    public boolean goal_reached() {
        if (getLocation().distanceTo(getGoal()) < 7) { 
            if (count_goal_reached == 0) {
                print_location("new");
                System.out.println(" ****** The robot has reached its goal !!! ******");
                System.out.println("The total distance traveled is " + cum_distance);
                System.out.println();
            }
            count_goal_reached++;
            return true;
        } else {
            return false;
        }
    }
    
    // Get the slope of the line connecting the start point and the goal 
    // point 
    public double getGoalSlope() {
        double slope;
        if (goal.getX() == start.getX()) {
            slope = 999999999;
        }else {
            slope = (goal.getY() - start.getY())/(goal.getX() - start.getX());
        }
        return slope;
    }
   
    // Get the slope of the line conecting the goal point and the 
    // current location of the robot
     public double getLocationSlope() {
        double slope_location;
        if (location.getX() == goal.getX()) {
            slope_location = 999999999;
        }else {
            slope_location = (location.getY() - goal.getY())/(location.getX() - goal.getX());    
        }
        return slope_location;
    }
  
    // Returns true if the mobile robot crosses one of the "hit" points 
    // previously visited 
    public boolean hp_visited() {
        Point temp_location = getLocation(); 
        boolean result = false;
        for(Point p : visited_points){
            if (temp_location.distanceTo(p) < 5) {  
                result = true; 
            }
        }
        return result;
    }
  
    // Get the "encoder" distance traveled by the mobile robot
    public double get_enc_distance() {
        int d1 = robot.getCount(0);
        int d2 = robot.getCount(1);
        double enc_dist = (d1 + d2)/2;
        enc_dist = -6*Math.PI*enc_dist/64;
        robot.resetCount();
        return enc_dist;
    }
   
    // Get the "encoder" rotation made by the robot
     public double get_enc_rotation() {
        int d1 = robot.getCount(0);
        int d2 = robot.getCount(1);
        double enc_rot = (Math.abs(d1) + Math.abs(d2))/2;
        enc_rot = (enc_rot/enc_full)*360;
        if (d2 < 0) {
            enc_rot = - enc_rot;
        }
        robot.resetCount();
        return enc_rot;
    }
    
    // Get the cumulative distance traveled by the mobile robot
    public double get_cum_distance() {
        return cum_distance; 
    }
    
    // Print the current location of the robot
    public void print_location(String current) {
        if (current.equals("initial")) {
            System.out.print("The initial ");
        }else if (current.equals("current")) {
            System.out.print("The current ");
        }else {
            System.out.print("The new ");
        }
        System.out.println("location and orientation of the robot are"); 
        System.out.print("(" + location.getX() + " cm," + location.getY() + " cm) ");
        System.out.println("and " + orientation + " degrees, respectively.");
    }
    
    // This class manages the speeds of the right and left wheels of 
    // the robot
    private class Speed {
        private double vr; // speed of the right wheel
        private double vl; // speed of the left wheel
        
        // Constructor for zero Speed:(0,0)
        public Speed() {
            vr = 0;
            vl = 0;
        }
      
        // Constructor for Speed:(speed_right_wheel, speed_left_wheel)
        public Speed(double vr_new, double vl_new) {
            vr = vr_new;
            vl = vl_new;
        }
        
        // Set the speed of the right wheel
        public void set_vr(double vr_new) {
            vr = vr_new;
        }
        
        // Get the speed of the right wheel
        public double get_vr() {
            return vr;
        }
        
        // Set the speed of the left wheel
        public void set_vl(double vl_new) {
            vl = vl_new;
        }
        
        // Get the speed of the left wheel
        public double get_vl() {
            return vl;
        }
    }   
           
}

// A point in the cartesian plane P:(x,y)
class Point {
    private double x;
    private double y;
    
    // Constructor for the origin point:(0,0)
    public Point() {
        x = 0;
        y = 0;
    }
    
    // Constructor for a Point:(x,y)
    public Point(double x_new, double y_new) {
        x = x_new;
        y = y_new;
    }
    
    // Set the x-coordinate of the point 
    public void setX(double x_new) {
        x = x_new;
    }
    
    // Get the x-coordinate of the point
    public double getX() {
        return x;
    }
    
    // Set the y-coordinate of the point
    public void setY(double y_new) {
        y = y_new;
    }
    
    // Get the y-coordinate of the point 
    public double getY() {
        return y;
    }
    
    // Get the distance of THIS point to another point
    public double distanceTo(Point point) {
        double delta_x = point.getX() - x;
        double delta_y = point.getY() - y;
        double distance = Math.sqrt(Math.pow(delta_x,2) + Math.pow(delta_y,2));
        return distance;
    }
}

