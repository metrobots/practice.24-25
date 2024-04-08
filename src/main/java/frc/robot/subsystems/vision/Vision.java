package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.utils.LimelightLib;

public class Vision extends SubsystemBase {

public static String aLN = "arm";
public static String fLN = "front";

//OK SO GUYS IF SOMETHING IS MARKED WITH AN A_ THAT MEANS THAT ITS FROM THE ARM LIMELIGHT
//IF ITS FROM THE INTAKE/FROM LIMELIGHT IT IS MARKED WITH A F_

//COORDINATE VALUE GETTERS
public static double a_tX = LimelightLib.getTX(aLN);
public static double a_tY = LimelightLib.getTY(aLN);
public static double a_tA = LimelightLib.getTA(aLN);
public static boolean a_tV = LimelightLib.getTV(aLN);

public static double f_tX = LimelightLib.getTX(fLN);
public static double f_tY = LimelightLib.getTY(fLN);
public static double f_tA = LimelightLib.getTA(fLN);
public static boolean f_tV = LimelightLib.getTV(fLN);

//ROBOT POSE GETTERS 
//3D
public static Pose3d a_botPose3d = LimelightLib.getBotPose3d(aLN);
public static double[] a_botPoseTargetSpace = LimelightLib.getBotPose_TargetSpace(aLN);
public static double[] a_botPose3dBlue = LimelightLib.getBotPose_wpiBlue(aLN);
public static double[] a_botPose3dRed = LimelightLib.getBotPose_wpiRed(aLN);

public static Pose3d f_botPose3d = LimelightLib.getBotPose3d(fLN);
public static double[] f_botPoseTargetSpace = LimelightLib.getBotPose_TargetSpace(fLN);
public static double[] f_botPose3dBlue = LimelightLib.getBotPose_wpiBlue(fLN);
public static double[] f_botPose3dRed = LimelightLib.getBotPose_wpiRed(fLN);
//2D
public static double[] a_botPose = LimelightLib.getBotPose(aLN);
public static Pose2d a_botPose2d = LimelightLib.getBotPose2d(aLN);
public static Pose2d a_botPose2dBlue = LimelightLib.getBotPose2d_wpiBlue(aLN);
public static Pose2d a_botPose2dRed = LimelightLib.getBotPose2d_wpiRed(aLN);

public static double[] f_botPose = LimelightLib.getBotPose(fLN);
public static Pose2d f_botPose2d = LimelightLib.getBotPose2d(fLN);
public static Pose2d f_botPose2dBlue = LimelightLib.getBotPose2d_wpiBlue(fLN);
public static Pose2d f_botPose2dRed = LimelightLib.getBotPose2d_wpiRed(fLN);

//CAMERA POSE GETTERS
public static Pose3d a_camPoseRobotSpace = LimelightLib.getCameraPose3d_RobotSpace(aLN);
public static Pose3d a_camPose3dRobotSpace = LimelightLib.getCameraPose3d_TargetSpace(aLN);
public static double[] a_camPose2dRobotSpace = LimelightLib.getCameraPose_TargetSpace(aLN);

public static Pose3d f_camPoseRobotSpace = LimelightLib.getCameraPose3d_RobotSpace(fLN);
public static Pose3d f_camPose3dRobotSpace = LimelightLib.getCameraPose3d_TargetSpace(fLN);
public static double[] f_camPose2dRobotSpace = LimelightLib.getCameraPose_TargetSpace(fLN);

//TARGET INFO GETTERS
public static Pose3d a_targetPose3dCamSpace = LimelightLib.getTargetPose3d_CameraSpace(aLN);
public static double a_targetID = LimelightLib.getFiducialID(aLN);
public static double[] a_targetColor = LimelightLib.getTargetColor(aLN);

public static Pose3d f_targetPose3dCamSpace = LimelightLib.getTargetPose3d_CameraSpace(fLN);
public static double f_targetID = LimelightLib.getFiducialID(fLN);
public static double[] f_targetColor = LimelightLib.getTargetColor(fLN);

// Extract the position of the target from the Pose3d object
public static Translation3d a_targetPosition = a_targetPose3dCamSpace.getTranslation();

public static Translation3d f_targetPosition = f_targetPose3dCamSpace.getTranslation();

// Calculate the distance between the robot and the target using the position of the target
double a_distance = Math.sqrt(Math.pow(a_targetPosition.getX(), 2) + Math.pow(a_targetPosition.getY(), 2) + Math.pow(a_targetPosition.getZ(), 2));

double f_distance = Math.sqrt(Math.pow(f_targetPosition.getX(), 2) + Math.pow(f_targetPosition.getY(), 2) + Math.pow(f_targetPosition.getZ(), 2));



//RANDOM OTHER SHIT GETTERS
public static double a_pipelineIndex = LimelightLib.getCurrentPipelineIndex(aLN);
public static double a_captureLatency = LimelightLib.getLatency_Capture(aLN);
public static double a_pipelineLatency = LimelightLib.getLatency_Pipeline(aLN);

public static double f_pipelineIndex = LimelightLib.getCurrentPipelineIndex(fLN);
public static double f_captureLatency = LimelightLib.getLatency_Capture(fLN);
public static double f_pipelineLatency = LimelightLib.getLatency_Pipeline(fLN);







  /**
   * CALCULATE THE DISTANCE TO THE NEAREST APRILTAG
   *
   * @param ty        THE CURRENT TY VALUE (USE A SUPPLIER)
   * @param armAngle  THE CURRENT ARM ANGLE VALUE (USE A SUPPLIER)
   * @param targetID  THE CURRENT TARGET ID (USE A SUPPLIER)
   * @param type      THE TYPE OF APRILTAG ("speaker", "amp", "hp-station", "stage", "any")
   */
//FUNCTION FOR CALCULATING APRILTAG DISTANCE
public static double calculateDistance(double ty, double armAngle, double targetID, String type) {

  //CONSTANTS
  double armLength = 12; //PLACEHOLDER VALUE
  double pivotOffset = 12; //PLACEHOLDER VALUE
  double limelightCenter = 12;
  double goalHeightInches = 12;;

  //CALCULATIONS
  double angleInRadians = Math.toRadians(armAngle); //CONVERT THE ANGLE TO RADIANS
  double verticalComponent = armLength * Math.cos(angleInRadians); //CALCULATE VERTICAL COMPONENT OF TRIANGLE
  double limelightLensHeightInches = verticalComponent + pivotOffset + limelightCenter; //FIND THE HEIGHT OF THE LIMELIGHT LENS FROM THE FLOOR

  double targetOffsetAngle_Vertical = ty;
  double limelightMountAngleDegrees = armAngle + 12; //CHANGE THIS VALUE WHEN WE MEASURE LIMELIGHT ANGLE

  double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
  double distanceInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

  double distanceFeet = distanceInches/12;
  // double distanceMeters = distanceFeet/3.281;
  // double distanceYards = distanceFeet/3;

  return distanceFeet;
  //return distanceMeters;
  //distanceYards;

  
}



Vision() {
  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}