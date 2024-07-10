package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.LimelightLib;
import java.util.logging.Level;
import java.util.logging.Logger;

public class Vision extends SubsystemBase {

    private static final Logger LOGGER = Logger.getLogger(Vision.class.getName());

    private static final String LIMELIGHT = "limelight";

    // Coordinate value getters
    private double tX;
    private double tY;
    private double tA;
    private boolean tV;

    // Robot pose getters
    private Pose3d botPose3d;
    private double[] botPoseTargetSpace;
    private double[] botPose3dBlue;
    private double[] botPose3dRed;
    private double[] botPose;
    private Pose2d botPose2d;
    private Pose2d botPose2dBlue;
    private Pose2d botPose2dRed;

    // Camera pose getters
    private Pose3d camPoseRobotSpace;
    private Pose3d camPose3dRobotSpace;
    private double[] camPose2dRobotSpace;

    // Target info getters
    private Pose3d targetPose3dCamSpace;
    private double targetID;
    private double[] targetColor;
    private Translation3d targetPosition;

    // Latency getters
    private double pipelineIndex;
    private double captureLatency;
    private double pipelineLatency;

    public Vision() {
        updateValues();
    }

    private void updateValues() {
        try {
            // Coordinate values
            tX = LimelightLib.getTX(LIMELIGHT);
            tY = LimelightLib.getTY(LIMELIGHT);
            tA = LimelightLib.getTA(LIMELIGHT);
            tV = LimelightLib.getTV(LIMELIGHT);

            // Robot pose values
            botPose3d = LimelightLib.getBotPose3d(LIMELIGHT);
            botPoseTargetSpace = LimelightLib.getBotPose_TargetSpace(LIMELIGHT);
            botPose3dBlue = LimelightLib.getBotPose_wpiBlue(LIMELIGHT);
            botPose3dRed = LimelightLib.getBotPose_wpiRed(LIMELIGHT);
            botPose = LimelightLib.getBotPose(LIMELIGHT);
            botPose2d = LimelightLib.getBotPose2d(LIMELIGHT);
            botPose2dBlue = LimelightLib.getBotPose2d_wpiBlue(LIMELIGHT);
            botPose2dRed = LimelightLib.getBotPose2d_wpiRed(LIMELIGHT);

            // Camera pose values
            camPoseRobotSpace = LimelightLib.getCameraPose3d_RobotSpace(LIMELIGHT);
            camPose3dRobotSpace = LimelightLib.getCameraPose3d_TargetSpace(LIMELIGHT);
            camPose2dRobotSpace = LimelightLib.getCameraPose_TargetSpace(LIMELIGHT);

            // Target info values
            targetPose3dCamSpace = LimelightLib.getTargetPose3d_CameraSpace(LIMELIGHT);
            targetID = LimelightLib.getFiducialID(LIMELIGHT);
            targetColor = LimelightLib.getTargetColor(LIMELIGHT);
            targetPosition = targetPose3dCamSpace.getTranslation();

            // Latency values
            pipelineIndex = LimelightLib.getCurrentPipelineIndex(LIMELIGHT);
            captureLatency = LimelightLib.getLatency_Capture(LIMELIGHT);
            pipelineLatency = LimelightLib.getLatency_Pipeline(LIMELIGHT);

        } catch (Exception e) {
            LOGGER.log(Level.SEVERE, "Failed to update values from Limelight", e);
        }
    }

    public void createVisionDashboard() {
        SmartDashboard.putNumber("Latency", pipelineLatency);
        SmartDashboard.putString("Pipeline", getPipelineName((int) pipelineIndex));
    }

    private String getPipelineName(int index) {
        switch (index) {
            case 1: return "Neural Net";
            case 2: return "Apriltags";
            case 3: return "Color Filtering";
            case 4: return "Driving";
            default: return "Unknown";
        }
    }

    @Override
    public void periodic() {
        updateValues(); // Update values periodically
    }
}
