//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.


package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.drivetrain.DriveSubsystem;
import frc.robot.utils.Constants.ControllerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;


public class RobotContainer {

  //SUBSYSTEMS
  private final DriveSubsystem drivetrain = new DriveSubsystem();


  //DRIVER CONTROLLERS
  public static CommandXboxController primaryDriver = new CommandXboxController(0);
  public static CommandXboxController secondaryDriver = new CommandXboxController(1);

  public static DoubleSolenoid  wrongSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);

  

  //SENDABLECHOOSER FOR AUTO
  private SendableChooser<Command> autoChooser;

  public RobotContainer() {

    //REGISTER THE COMMANDS BEFORE CREATING THE POSE ESTIMATOR


    configureButtonBindings(); //CONFIGURE BINDINGS

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    PortForwarder.add(5800, "limelight-arm.local", 5800);
    PortForwarder.add(5800, "limelight-front.local", 5800);

    ///CONFIGURE DEFAULT COMMANDS
    drivetrain.setDefaultCommand(

        //LEFT STICK IS TRANSLATION RIGHT STICK IS TURNING
      new RunCommand(() -> drivetrain.drive(
        -MathUtil.applyDeadband(primaryDriver.getLeftY(), ControllerConstants.driveDeadzone), //CONTROL THE ROBOT X SPEED
        -MathUtil.applyDeadband(primaryDriver.getLeftX(), ControllerConstants.driveDeadzone), //CONTROL THE ROBOT Y SPEED
        -MathUtil.applyDeadband(primaryDriver.getRightX(), ControllerConstants.driveDeadzone), //CONTROL THE ROBOT ROTATION
        false, true),
    drivetrain));
  }




  private void configureButtonBindings() {
    //DEFINE ALL OF THE BUTTON BINDINGS HERE PLEASE AND THANKS


}
    


  //THIS IS ALL OF THE AUTO PLEASE DON'T WRITE AUTO ANYWHERE ELSE
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}