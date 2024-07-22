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

    @Override
    public void periodic() {
        updateValues(); // Update values periodically
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

        // Displaying other relevant data on the dashboard
        SmartDashboard.putNumber("tX", tX);
        SmartDashboard.putNumber("tY", tY);
        SmartDashboard.putNumber("tA", tA);
        SmartDashboard.putBoolean("tV", tV);

        // Add robot pose values to SmartDashboard
        SmartDashboard.putString("Bot Pose 3D", botPose3d.toString());
        SmartDashboard.putString("Bot Pose Target Space", arrayToString(botPoseTargetSpace));
        SmartDashboard.putString("Bot Pose 3D Blue", arrayToString(botPose3dBlue));
        SmartDashboard.putString("Bot Pose 3D Red", arrayToString(botPose3dRed));
        SmartDashboard.putString("Bot Pose 2D", botPose2d.toString());
        SmartDashboard.putString("Bot Pose 2D Blue", botPose2dBlue.toString());
        SmartDashboard.putString("Bot Pose 2D Red", botPose2dRed.toString());

        // Add camera pose values to SmartDashboard
        SmartDashboard.putString("Cam Pose Robot Space", camPoseRobotSpace.toString());
        SmartDashboard.putString("Cam Pose 3D Robot Space", camPose3dRobotSpace.toString());
        SmartDashboard.putString("Cam Pose 2D Robot Space", arrayToString(camPose2dRobotSpace));

        // Add target info values to SmartDashboard
        SmartDashboard.putString("Target Pose 3D Cam Space", targetPose3dCamSpace.toString());
        SmartDashboard.putNumber("Target ID", targetID);
        SmartDashboard.putString("Target Color", arrayToString(targetColor));
        SmartDashboard.putString("Target Position", targetPosition.toString());
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

    private String arrayToString(double[] array) {
        if (array == null) return "null";
        StringBuilder sb = new StringBuilder();
        for (double v : array) {
            sb.append(v).append(" ");
        }
        return sb.toString().trim();
    }

    // Getters for the private fields
    public double getTX() { return tX; }
    public double getTY() { return tY; }
    public double getTA() { return tA; }
    public boolean isTV() { return tV; }
    public Pose3d getBotPose3d() { return botPose3d; }
    public double[] getBotPoseTargetSpace() { return botPoseTargetSpace; }
    public double[] getBotPose3dBlue() { return botPose3dBlue; }
    public double[] getBotPose3dRed() { return botPose3dRed; }
    public double[] getBotPose() { return botPose; }
    public Pose2d getBotPose2d() { return botPose2d; }
    public Pose2d getBotPose2dBlue() { return botPose2dBlue; }
    public Pose2d getBotPose2dRed() { return botPose2dRed; }
    public Pose3d getCamPoseRobotSpace() { return camPoseRobotSpace; }
    public Pose3d getCamPose3dRobotSpace() { return camPose3dRobotSpace; }
    public double[] getCamPose2dRobotSpace() { return camPose2dRobotSpace; }
    public Pose3d getTargetPose3dCamSpace() { return targetPose3dCamSpace; }
    public double getTargetID() { return targetID; }
    public double[] getTargetColor() { return targetColor; }
    public Translation3d getTargetPosition() { return targetPosition; }
    public double getPipelineIndex() { return pipelineIndex; }
    public double getCaptureLatency() { return captureLatency; }
    public double getPipelineLatency() { return pipelineLatency; }
}
