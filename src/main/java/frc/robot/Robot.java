//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.


package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;




/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Command autonomousCommand;
  private RobotContainer robotContainer; 
  


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    //Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    //autonomous chooser on the dashboard.
    robotContainer = new RobotContainer();

    //chooser.setDefaultOption("Default Auto", defaultAuto);
    //chooser.addOption("My Auto 1", customAuto1);
    //chooser.addOption("My Auto 2", customAuto2);
    //chooser.addOption("My Auto 3", customAuto3);
    //chooser.addOption("My Auto 4", customAuto4);
    //chooser.addOption("My Auto 5", customAuto5);
    //chooser.addOption("My Auto 6", customAuto6);
    //SmartDashboard.putData("Auto Choices", chooser);
   
  }


 
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }


  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }


  @Override
  public void disabledPeriodic() {
    
  }


  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    autonomousCommand = robotContainer.getAutonomousCommand();


    if (autonomousCommand != null) {
      autonomousCommand.schedule();
      }


    //autoSelected = chooser.getSelected();
    //System.out.println("Auto Selected: " + autoSelected);
  }










 
  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {




  }


  @Override
  public void teleopInit() {
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
  }


  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}


  @Override
  public void testInit() {
    //Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }


  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}