// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.pid_subsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PID_Subsystem extends SubsystemBase {
  /** Creates a new PID_Subsystem. */

  //Create the three constants for the PID controller
  public static double exampleP = 0;
  public static double exampleI = 0;
  public static double exampleD = 0;

  //Create a move value to store the PID calculated number
  public static double exampleMoveValue;

  //Create the setpoint for the subsystem to move to. It can be in any measurement (degrees, radians, meters)
  public static double exampleSetpoint;

  //This is a placeholder for a sensor reading, such as a encoder or similar sensor
  public static double exampleSensorReading;

  //This creates a PIDController with the specified constants
  public static PIDController exampleController = new PIDController(exampleP, exampleI, exampleD);


  public PID_Subsystem() {

  }

  @Override
  public void periodic() {
    //Update the sensor value (It says it's useless because this doesn't actually update a sensor value)
    exampleSensorReading = exampleSensorReading;

    //Calculate the current PID value needed to move until the sensor reading matches the setpoint
    exampleMoveValue = exampleController.calculate(exampleSensorReading, exampleSetpoint);

    //Example function to move something using the sensor value
    exampleMoveFunction(exampleMoveValue);
  }

  public static void exampleMoveFunction(double moveValue) {
    
  }
}
