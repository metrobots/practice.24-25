  //Copyright (c) FIRST and other WPILib contributors.
  //Open Source Software; you can modify and/or share it under the terms of
  //the WPILib BSD license file in the root directory of this project.
  package frc.robot.utils;

  import edu.wpi.first.wpilibj2.command.SubsystemBase;
  import frc.robot.subsystems.drivetrain.DriveSubsystem;
  import edu.wpi.first.wpilibj.RobotController;
  import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
  import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
  import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
  import edu.wpi.first.cameraserver.CameraServer;
  import edu.wpi.first.wpilibj.DriverStation;
  import edu.wpi.first.wpilibj.PowerDistribution;
  import java.util.Map;



  public class CreateDashboard extends SubsystemBase {

    PowerDistribution pdp = new PowerDistribution();
    
    //CREATES A NEW DASHBOARD.
    public CreateDashboard() {

    }

    @Override
    public void periodic() {

    }


    public void displayPosition() {
    SmartDashboard.putNumber("xPos", DriveSubsystem.x);
    SmartDashboard.putNumber("yPos", DriveSubsystem.y);
    }

    public void displayGyroAngle() {
      SmartDashboard.putNumber("Angle", DriveSubsystem.gyro.getAngle());
    }

    public void displayBatteryVoltage() {
      SmartDashboard.putNumber("Voltage",RobotController.getBatteryVoltage());
    }

    public void displayRobotConnected() {
      SmartDashboard.putBoolean("Connected",DriverStation.isDSAttached());
    }

    public void displayCurrent(int channel) {
      
      SmartDashboard.putNumber("Current", pdp.getCurrent(channel));

    }

    public void displayLimelightStream() {
        // Start the camera server
        CameraServer.startAutomaticCapture();

        // Add the Limelight stream URL to Shuffleboard
        String limelightURL = "http://limelight.local:5801"; // Replace with your Limelight's IP address if different

        // Use Shuffleboard to display the stream
        Shuffleboard.getTab("Limelight").add("Limelight Stream", limelightURL)
            .withWidget(BuiltInWidgets.kCameraStream)
            .withProperties(Map.of(
                "Show Crosshair", true,
                "Show Controls", false,
                "Show Timestamp", true
            ));
    }
    







  }

    


