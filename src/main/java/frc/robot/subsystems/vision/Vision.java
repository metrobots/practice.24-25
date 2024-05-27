package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.LimelightLib;

public class Vision extends SubsystemBase {

public static String limelight = "limelight";


//OK SO GUYS IF SOMETHING IS MARKED WITH AN  THAT MEANS THAT ITS FROM THE ARM LIMELIGHT

//COORDINATE VALUE GETTERS
public static double tX = LimelightLib.getTX(limelight);
public static double tY = LimelightLib.getTY(limelight);
public static double tA = LimelightLib.getTA(limelight);
public static boolean tV = LimelightLib.getTV(limelight);

//ROBOT POSE GETTERS 
//3D
public static Pose3d botPose3d = LimelightLib.getBotPose3d(limelight);
public static double[] botPoseTargetSpace = LimelightLib.getBotPose_TargetSpace(limelight);
public static double[] botPose3dBlue = LimelightLib.getBotPose_wpiBlue(limelight);
public static double[] botPose3dRed = LimelightLib.getBotPose_wpiRed(limelight);

//2D
public static double[] botPose = LimelightLib.getBotPose(limelight);
public static Pose2d botPose2d = LimelightLib.getBotPose2d(limelight);
public static Pose2d botPose2dBlue = LimelightLib.getBotPose2d_wpiBlue(limelight);
public static Pose2d botPose2dRed = LimelightLib.getBotPose2d_wpiRed(limelight);

//CAMERA POSE GETTERS
public static Pose3d camPoseRobotSpace = LimelightLib.getCameraPose3d_RobotSpace(limelight);
public static Pose3d camPose3dRobotSpace = LimelightLib.getCameraPose3d_TargetSpace(limelight);
public static double[] camPose2dRobotSpace = LimelightLib.getCameraPose_TargetSpace(limelight);

//TARGET INFO GETTERS
public static Pose3d targetPose3dCamSpace = LimelightLib.getTargetPose3d_CameraSpace(limelight);
public static double targetID = LimelightLib.getFiducialID(limelight);
public static double[] targetColor = LimelightLib.getTargetColor(limelight);

// Extract the position of the target from the Pose3d object
public static Translation3d targetPosition = targetPose3dCamSpace.getTranslation();


// Calculate the distance between the robot and the target using the position of the target
double distance = Math.sqrt(Math.pow(targetPosition.getX(), 2) + Math.pow(targetPosition.getY(), 2) + Math.pow(targetPosition.getZ(), 2));

//RANDOM OTHER STUFF GETTERS
public static double pipelineIndex = LimelightLib.getCurrentPipelineIndex(limelight);
public static double captureLatency = LimelightLib.getLatency_Capture(limelight);
public static double pipelineLatency = LimelightLib.getLatency_Pipeline(limelight);


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

public static void createVisionDashboard() {
  SmartDashboard.putNumber("Latency", pipelineLatency);

  if (pipelineIndex == 1) {
    SmartDashboard.putString("Pipeline:", "Neural Net");
  } else if (pipelineIndex == 2) {
    SmartDashboard.putString("Pipeline:", "Apriltags");
  } else if (pipelineIndex ==  3) {
    SmartDashboard.putString("Pipeline:", "Color Filtering");
  } else if (pipelineIndex == 4) {
    SmartDashboard.putString("Pipeline:", "Driving");
  }

  
  
}



Vision() {
  
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}