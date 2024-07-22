//Copyright (c) FIRST and other WPILib contributors.
//Open Source Software; you can modify and/or share it under the terms of
//the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems.drivetrain;

import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.WPIUtilJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SPI;
import frc.robot.utils.SwerveUtils;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DriveSubsystem extends SubsystemBase {
  //CREATE SWERVE MODULES

  public final SwerveDrivePoseEstimator swerveDrivePoseEstimator;
  public Rotation2d getHeadingPose2d;

  //THIS IS THE FRONT LEFT MODULE
  private final static SwerveModule frontLeft = new SwerveModule(
      DriveConstants.frontLeftDrivingCanId,
      DriveConstants.frontLeftTurningCanId,
      DriveConstants.frontLeftChassisAngularOffset);


  //THIS IS THE FRONT RIGHT MODULE
  private final static SwerveModule frontRight = new SwerveModule(
      DriveConstants.frontRightDrivingCanId,
      DriveConstants.frontRightTurningCanId,
      DriveConstants.frontRightChassisAngularOffset);


  //THIS IS THE BACK LEFT MODULE
  private final static SwerveModule rearLeft = new SwerveModule(
      DriveConstants.rearLeftDrivingCanId,
      DriveConstants.rearLeftTurningCanId,
      DriveConstants.backLeftChassisAngularOffset);


  //THIS IS THE BACK RIGHT MODULE
  private final static SwerveModule rearRight = new SwerveModule(
      DriveConstants.rearRightDrivingCanId,
      DriveConstants.rearRightTurningCanId,
      DriveConstants.backRightChassisAngularOffset);


  //GYRO THIS IS WHERE THE GYRO GOES (CURRENTLY THIS IS SAYING THAT IT IS A NAVX MOUNTED TO THE TOP PART OF THE RIO)
  public static final AHRS gyro = new AHRS(SPI.Port.kMXP);


  //I DON'T EVEN KNOW WHAT SLEW RATE IS BUT THESE CONTROL SLEW RATE AND THE DOCS TOLD ME TO (LATERAL MOVEMENT MAYBE?)
  private double currentRotation = 0.0;
  private double currentTranslationDir = 0.0;
  private double currentTranslationMag = 0.0;

  private SlewRateLimiter magLimiter = new SlewRateLimiter(DriveConstants.magnitudeSlewRate);   //LIMITS THE RATE OF CHANGE OF THE MAGNITUDE OF THE ROBOT'S SPEED
  private SlewRateLimiter rotLimiter = new SlewRateLimiter(DriveConstants.rotationalSlewRate);  //LIMITS THE RATE OF CHANGE OF THE ROTATION SPEED OF THE ROBOT
  private double prevTime = WPIUtilJNI.now() * 1e-6;


  //TRACKING ROBOT POSE
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
      DriveConstants.DriveKinematics,
      Rotation2d.fromDegrees(gyro.getAngle()),
      new SwerveModulePosition[] {
          frontLeft.getPosition(),
          frontRight.getPosition(),
          rearLeft.getPosition(),
          rearRight.getPosition()
      });

      public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] modulePositions = {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        };
        return modulePositions;
    }



  //THIS IS THE CHOOSER FOR THE AUTO OPTIONS


  //CREATES A NEW DRIVESUBSYSTEM.
  public DriveSubsystem() {

    getHeadingPose2d = Rotation2d.fromDegrees(getHeading());

    
    swerveDrivePoseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.DriveKinematics, 
      getHeadingPose2d, 
      getModulePositions(), 
      new Pose2d(new Translation2d(0, 0), 
      Rotation2d.fromDegrees(0))); 

      AutoBuilder.configureHolonomic(
        this::getPose, //POSE SUPPLIER
        this::resetOdometry, //METHOD TO RESET ODOMETRY (IF IT HAS A STARTING POS)
        this::getRobotRelativeSpeeds, //METHOD TO GET CHASSIS SPEEDS FROM THE STATE OF EACH MODULE
        this::driveRobotRelative, //METHOD TO DRIVE THE ROBOT TO SPECIFIC CHASSIS SPEEDS
        Constants.DriveConstants.holonomicPathFollowerConfig, //PATH FOLLOWER CONFIG
        () -> {
          //BOOL SUPPLIER CONTROLS WHEN THE PATH IS MIRRORED TO THE RED SIDE
          //THIS WILL FLIP THE PATH BEING FOLLOWED TO THE RED SIDE
          //THE ORIGIN WILL REMAIN ON THE BLUE SIDE
          var alliance = DriverStation.getAlliance();
          return alliance.map(a -> a == DriverStation.Alliance.Red).orElse(false);
        },
        this
);
        

  }

      
  public static double fR = frontRight.getRawTurnEncoder();
  public static double fL = frontLeft.getRawTurnEncoder();
  public static double bR = rearLeft.getRawTurnEncoder();
  public static double bL = rearRight.getRawTurnEncoder();
  public static double x;
  public static double y;
  public static Pose2d lmao;

  
  @Override
  public void periodic() {

    getHeadingPose2d = Rotation2d.fromDegrees(getHeading());

    swerveDrivePoseEstimator.update(getHeadingPose2d, getModulePositions()); //THIS ONE UPDATES THE ESTIMATED POSE OF SWERVE

    @SuppressWarnings("unused")
    double currentHeading = getHeading(); //SET HEADING ON SMARTDASHBOARD
    // SmartDashboard.putNumber("Heading", currentHeading); 


    //UPDATE ODOMETRY
    odometry.update(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        });

    //INSTRUCTIONS - PHYSICALLY TURN ALL OF THE WHEELS SO THAT THEY FACE FORWARD. THEN IN THE CONSTANT FILE SET ALL CHASSIS ANGULAR OFFSETS TO WHATEVER VALUE THE RESPECTIVE MODULE IS READING

    x = swerveDrivePoseEstimator.getEstimatedPosition().getX();
    y = swerveDrivePoseEstimator.getEstimatedPosition().getY();

    // SmartDashboard.putNumber("Front Left Module Angle:", frontLeft.getRawTurnEncoder());
    // SmartDashboard.putNumber("Front Right Module Angle:", frontRight.getRawTurnEncoder());
    // SmartDashboard.putNumber("Back Left Module Angle:", rearLeft.getRawTurnEncoder());
    // SmartDashboard.putNumber("Back Right Module Angle:", rearRight.getRawTurnEncoder());


  }



  /**
   * RETURNS THE ROBOT POSE
   *
   * @return THE POSE
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
    // return swerveDrivePoseEstimator.getEstimatedPosition();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    // Retrieve the current states of each swerve module
    SwerveModuleState frontLeftState = frontLeft.getState();
    SwerveModuleState frontRightState = frontRight.getState();
    SwerveModuleState rearLeftState = rearLeft.getState();
    SwerveModuleState rearRightState = rearRight.getState();

    // Convert the swerve module states to chassis speeds
    return Constants.DriveConstants.DriveKinematics.toChassisSpeeds(
        frontLeftState, frontRightState, rearLeftState, rearRightState);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    // Convert the robot-relative speeds to swerve module states
    SwerveModuleState[] moduleStates = Constants.DriveConstants.DriveKinematics.toSwerveModuleStates(speeds);

    // Desaturate the wheel speeds to ensure they are within the maximum speed
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.DriveConstants.maxSpeedMetersPerSecond);

    // Set the desired state for each swerve module
    setModuleStates(moduleStates);
  }




  /**
   * RESETS ODOMETRY TO SPECIFIED POSE
   *
   * @param pose THE POSE TO SET ODOMETRY TO
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(
        Rotation2d.fromDegrees(gyro.getAngle()),
        new SwerveModulePosition[] {
            frontLeft.getPosition(),
            frontRight.getPosition(),
            rearLeft.getPosition(),
            rearRight.getPosition()
        },
        pose);
  }


  /**
   * DRIVE ROBOT USING JOYSTICK INPUT
   *
   * @param xSpeed        SPEED OF THE ROBOT IN THE X DIRECTION (FORWARD/BACK)
   * @param ySpeed        SPEED OF THE ROBOT IN THE Y DIRECTION (SIDEWAYS)
   * @param rot           ANGULAR RATE OF THE ROBOT
   * @param fieldRelative WHETHER THE PROVIDED SPEEDS ARE ROBOT RELATIVE
   * @param rateLimit     WHETHER TO ENABLE RATE LIMITING FOR EASIER CONTROL
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean rateLimit) {
   
    double xSpeedCommanded;
    double ySpeedCommanded;


    if (rateLimit) {
      //CONVERT XY VALUES TO POLAR FOR RATE LIMITING
      double inputTranslationDir = Math.atan2(ySpeed, xSpeed);
      double inputTranslationMag = Math.sqrt(Math.pow(xSpeed, 2) + Math.pow(ySpeed, 2));


      //CALCULATE LEW RATE BASED ON ESTIMATE OF LATERAL ACCELERATION
      double directionSlewRate;
      if (currentTranslationMag != 0.0) {
        directionSlewRate = Math.abs(DriveConstants.directionSlewRate / currentTranslationMag);
      } else {
        directionSlewRate = 500.0; //HIGH NUMBER TO MAKE IT ALMOST INSTANTANEOUS
      }
     


      double currentTime = WPIUtilJNI.now() * 1e-6;
      double elapsedTime = currentTime - prevTime;
      double angleDif = SwerveUtils.AngleDifference(inputTranslationDir, currentTranslationDir);
      if (angleDif < 0.45*Math.PI) {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(inputTranslationMag);
      }
      else if (angleDif > 0.85*Math.PI) {
        if (currentTranslationMag > 1e-4) { //TINY NUMBER TO AVOID FLOATING POINT ERRORS
          //KEEP CURRENT TRANSLATION DIR THE SAME
          currentTranslationMag = magLimiter.calculate(0.0);
        }
        else {
          currentTranslationDir = SwerveUtils.WrapAngle(currentTranslationDir + Math.PI);
          currentTranslationMag = magLimiter.calculate(inputTranslationMag);
        }
      }
      else {
        currentTranslationDir = SwerveUtils.StepTowardsCircular(currentTranslationDir, inputTranslationDir, directionSlewRate * elapsedTime);
        currentTranslationMag = magLimiter.calculate(0.0);
      }
      prevTime = currentTime;
     
      xSpeedCommanded = currentTranslationMag * Math.cos(currentTranslationDir);
      ySpeedCommanded = currentTranslationMag * Math.sin(currentTranslationDir);
      currentRotation = rotLimiter.calculate(rot);




    } else {
      xSpeedCommanded = xSpeed;
      ySpeedCommanded = ySpeed;
      currentRotation = rot;
    }


    //CONVERT COMMAND SPEEDS INTO DRIVETRAIN READY SPEEDS
    double xSpeedDelivered = xSpeedCommanded * DriveConstants.maxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeedCommanded * DriveConstants.maxSpeedMetersPerSecond;
    double rotDelivered = currentRotation * DriveConstants.maxAngularSpeed;


    var swerveModuleStates = DriveConstants.DriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(
        swerveModuleStates, DriveConstants.maxSpeedMetersPerSecond);
    frontLeft.setDesiredState(swerveModuleStates[0]);
    frontRight.setDesiredState(swerveModuleStates[1]);
    rearLeft.setDesiredState(swerveModuleStates[2]);
    rearRight.setDesiredState(swerveModuleStates[3]);
  }


  //SET WHEELS INTO X FORMATION TO STOP MOVEMENT
  public void setWheelsX() {
    frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }


  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates THE DESIRED MODULE STATES
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.maxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    rearLeft.setDesiredState(desiredStates[2]);
    rearRight.setDesiredState(desiredStates[3]);
  }


  //RESETS ENCODERS TO READ 0
  public void resetEncoders() {
    frontLeft.resetEncoders();
    rearLeft.resetEncoders();
    frontRight.resetEncoders();
    rearRight.resetEncoders();
  }


  //ZEROS HEADING OF ROBOT
  public void zeroHeading() {
    gyro.reset();
  }


  /**
   * RETURNS THE HEADING
   *
   * @return THE ROBOT HEADING (-180 to 180)
   */
  public double getHeading() {
    // return Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();

      double rawAngle = Rotation2d.fromDegrees(gyro.getAngle()).getDegrees();
      
      // Use angleModulus to wrap the angle between -180 and 180 degrees
      double wrappedAngle = Math.toDegrees(MathUtil.angleModulus(Math.toRadians(rawAngle)));
  
      return wrappedAngle;
  }



  /**
   * RETURNS THE TURN RATE
   *
   * @return THE TURN RATE FOR THE ROBOT IN DEGREES PER SECOND
   */
  public double getTurnRate() {
    return gyro.getRate() * (DriveConstants.gyroReversed ? -1.0 : 1.0);
  }

  public void recalibrateGyro() {
    gyro.reset();
}

}