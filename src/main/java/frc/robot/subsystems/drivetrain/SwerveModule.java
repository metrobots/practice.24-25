//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.drivetrain;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import frc.robot.subsystems.drivetrain.SwerveModule;
import frc.robot.utils.Constants.ModuleConstants;


public class SwerveModule {
  //THESE ARE THE SPARKMAXES
  public final CANSparkMax drivingSparkMax;
  private final CANSparkMax turningSparkMax;


  //THESE ARE THE ENCODERS
  private final RelativeEncoder drivingEncoder;
  private final AbsoluteEncoder turningEncoder;


  //THESE ARE THE PID CONTROLLERS
  private final SparkPIDController drivingPIDController;
  private final SparkPIDController turningPIDController;


  private double chassisAngularOffset = 0;
  private SwerveModuleState desiredState = new SwerveModuleState(0.0, new Rotation2d());




  //CREATES THE MAXSWERVEMODULE AND CONFIGURES EVERYTHING
  public SwerveModule(int drivingCANId, int turningCANId, double chassisAngularOffset) {
    drivingSparkMax = new CANSparkMax(drivingCANId, MotorType.kBrushless);
    turningSparkMax = new CANSparkMax(turningCANId, MotorType.kBrushless);


    //FACTORY RESET SO WE GET THE SPARKMAXES TO A KNOWN STATE BEFORE CONFIGURING THEM
    drivingSparkMax.restoreFactoryDefaults();
    turningSparkMax.restoreFactoryDefaults();


    //SETUP ENCODERS AND PID CONTROLLERS FOR THE DRIVING AND TURNING SPARKS MAX
    drivingEncoder = drivingSparkMax.getEncoder();
    turningEncoder = turningSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    drivingPIDController = drivingSparkMax.getPIDController();
    turningPIDController = turningSparkMax.getPIDController();
    drivingPIDController.setFeedbackDevice(drivingEncoder);
    turningPIDController.setFeedbackDevice(turningEncoder);


    //APPLY POSITION AND VELOCITY CONVERSION FACTOR TO DRIVING ENCODER FOR M AND M/S FOR WPI SWERVE API
    drivingEncoder.setPositionConversionFactor(ModuleConstants.drivingEncoderPositionFactor);
    drivingEncoder.setVelocityConversionFactor(ModuleConstants.drivingEncoderVelocityFactor);


    //APPLY POSITION AND VELOCITY CONVERSION FACTORS TO TURNING ENCODER FOR WPI SWERVE API
    turningEncoder.setPositionConversionFactor(ModuleConstants.turningEncoderPositionFactor);
    turningEncoder.setVelocityConversionFactor(ModuleConstants.turningEncoderVelocityFactor);


    //INVERT TURNING ENCODER (DON'T ASK ME I DON'T KNOW WHY)
    turningEncoder.setInverted(ModuleConstants.turningEncoderInverted);


    //ENABLE PID WRAPAROUND (350° TO 10°)
    turningPIDController.setPositionPIDWrappingEnabled(true);
    turningPIDController.setPositionPIDWrappingMinInput(ModuleConstants.turningEncoderPositionPIDMinInput);
    turningPIDController.setPositionPIDWrappingMaxInput(ModuleConstants.turningEncoderPositionPIDMaxInput);




    //HEY ROOKIE PROGRAMMERS (AND MATEO) YOU NEED TO CHANGE THESE VALUES FOR EVERY DIFFERENT ROBOT


    //SET PID GAINS FOR DRIVING MOTOR (NOT FINAL VALUES MAKE SURE TO CHANGE)
    //ASK MATEO FOR PID TUNING PROCEDURES
    drivingPIDController.setP(ModuleConstants.drivingP);
    drivingPIDController.setI(ModuleConstants.drivingI);
    drivingPIDController.setD(ModuleConstants.drivingD);
    drivingPIDController.setFF(ModuleConstants.drivingFF);
    drivingPIDController.setOutputRange(ModuleConstants.drivingMinOutput,
        ModuleConstants.drivingMaxOutput);


    //SET PID GAINS FOR TURNING MOTOR (NOT FINAL VALUES MAKE SURE TO CHANGE)
    turningPIDController.setP(ModuleConstants.turningP);
    turningPIDController.setI(ModuleConstants.turningI);
    turningPIDController.setD(ModuleConstants.turningD);
    turningPIDController.setFF(ModuleConstants.turningFF);
    turningPIDController.setOutputRange(ModuleConstants.turningMinOutput,
        ModuleConstants.turningMaxOutput);


    drivingSparkMax.setIdleMode(ModuleConstants.drivingMotorIdleMode);
    turningSparkMax.setIdleMode(ModuleConstants.turningMotorIdleMode);
    drivingSparkMax.setSmartCurrentLimit(ModuleConstants.drivingMotorCurrentLimit);
    turningSparkMax.setSmartCurrentLimit(ModuleConstants.turningMotorCurrentLimit);


    //SAVE SPARKMAX CONFIGS SO MAKE SURE BROWNOUTS DON'T AFFECT ANYTHING
    drivingSparkMax.burnFlash();
    turningSparkMax.burnFlash();


    this.chassisAngularOffset = chassisAngularOffset;
    desiredState.angle = new Rotation2d(turningEncoder.getPosition());
    drivingEncoder.setPosition(0);
  }

  //I FIXED THE ISSUE 😛😛😛😛😛 -MATEO AND RAHMA
  public double getRawTurnEncoder(){
    return turningEncoder.getPosition();
  } 

  /**
   * RETURNS THE CURRENT STATE OF THE MODULE
   *
   * @return THE CURRENT MODULE STATE
   */
  public SwerveModuleState getState() {
    //THIS ONE GETS 
    return new SwerveModuleState(drivingEncoder.getVelocity(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }


  /**
   * RETURNS CURRENT POSITION OF MODULE
   *
   * @return CURRENT POSITION OF MODULE
   */
  public SwerveModulePosition getPosition() {
    //APPLY CHASSIS ANGULAR OFFSET TO ENCODER TO GET RELATIVE POSITION
    return new SwerveModulePosition(
        drivingEncoder.getPosition(),
        new Rotation2d(turningEncoder.getPosition() - chassisAngularOffset));
  }


  /**
   * SETS THE DESIRED STATE
   *
   * @param desiredState DESIRED STATE WITH SPEED AND ANGLE
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    //APPLY CHASSIS ANGULAR OFFSET TO THE TARGET POSITION
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = desiredState.angle.plus(Rotation2d.fromRadians(chassisAngularOffset));


   
    //OPTIMIZE REFERENCE STATE TO STOP SPINNING > 90
    SwerveModuleState optimizedDesiredState = SwerveModuleState.optimize(correctedDesiredState,
        new Rotation2d(turningEncoder.getPosition()));


    //COMMAND SPARKMAXES TO SETPOINTS
    drivingPIDController.setReference(optimizedDesiredState.speedMetersPerSecond, CANSparkMax.ControlType.kVelocity);
    turningPIDController.setReference(optimizedDesiredState.angle.getRadians(), CANSparkMax.ControlType.kPosition);


    this.desiredState = desiredState;
    //DO NOT DELETE ITS NOT ACTUALLY USELESS SILLY
  }


  //ZERO ALL THE MODULE ENCODERS
  public void resetEncoders() {
    drivingEncoder.setPosition(0);
  }


 
}