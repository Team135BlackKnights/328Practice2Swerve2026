package frc.robot.subsystems;

import java.util.Optional;
import java.util.concurrent.atomic.AtomicBoolean;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers; // lots of code will likely be copied from the limelight vision docs so go there
import frc.robot.LoggableTunedNumber;
import frc.robot.RobotContainer;
import frc.robot.LimelightHelpers.PoseEstimate;

public class Vision extends SubsystemBase {

    double tx = LimelightHelpers.getTX("limelight");  // Horizontal offset from crosshair to target in degrees
    double ty = LimelightHelpers.getTY("limelight");  // Vertical offset from crosshair to target in degrees
    double ta = LimelightHelpers.getTA("limelight");  // Target area (0% to 100% of image)
    boolean hasTarget = LimelightHelpers.getTV("limelight"); // Do you have a valid target?

    double txnc = LimelightHelpers.getTXNC("limelight");  // Horizontal offset from principal pixel/point to target in degrees
    double tync = LimelightHelpers.getTYNC("limelight");  // Vertical  offset from principal pixel/point to target in degrees
    static double xPositionHubRelative = 0;
    static double yPositionHubRelative = 0;
    
        int[] validIDs = {1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28}; // ids to track
        boolean rejectUpdate = true;
    
        PoseEstimate mt2;
    
        @SuppressWarnings("")
        public Vision(){
            
            LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
    
            // Switch to pipeline 0
            LimelightHelpers.setPipelineIndex("limelight", 0);
    
            LimelightHelpers.setLEDMode_PipelineControl("limelight"); 
            //LimelightHelpers.setLEDMode_PipelineControl("limelight"); 
    
            // Set a custom crop window for improved performance (-1 to 1 for each value)
            LimelightHelpers.setCropWindow("limelight", -0.5, 0.5, -0.5, 0.5);
    
    
            // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
            LimelightHelpers.setCameraPose_RobotSpace("limelight", 
                0.4191,    // Forward offset (meters)
                -0.1524,    // Side offset (meters)
                0.2286,    // Height offset (meters)
                0.0,    // Roll (degrees)
                30.0,   // Pitch (degrees)
                0.0     // Yaw (degrees)
            );
    
            // Set AprilTag offset tracking point (meters)
            LimelightHelpers.setFiducial3DOffset("", 
                0.0,    // Forward offset
                0.0,    // Side offset  
                0.0     // Height offset
            );
    
            // Configure AprilTag detection
            LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution for improved framerate and reduced range
        }
    
        public void periodic(){
            
            LimelightHelpers.SetRobotOrientation("limelight", RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
            LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
            
            // if our angular velocity is greater than 360 degrees per second, ignore vision updates
            if(mt2 == null || mt2.tagCount < 1 || Math.abs(RobotContainer.gyro.getAngularVelocityYDevice().getValueAsDouble()) > 360){
                rejectUpdate = true;
            } else {
                rejectUpdate = false;
            } 
            
            if (!rejectUpdate) {
                RobotContainer.m_SwerveS.poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7,0.7,999));
                RobotContainer.m_SwerveS.poseEstimator.addVisionMeasurement(
                    mt2.pose,
                    mt2.timestampSeconds);
            }
    
            Logger.recordOutput("Vision/tx", LimelightHelpers.getTX("limelight"));
            Logger.recordOutput("Vision/ty", LimelightHelpers.getTY("limelight"));
            Logger.recordOutput("Vision/ta", LimelightHelpers.getTA("limelight"));
            Logger.recordOutput("Vision/robotPose", RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition());
            Logger.recordOutput("Vision/tagsSeen", (rejectUpdate ? 0 : mt2.tagCount));
            Logger.recordOutput("Vision/xPositionHubRelative", Constants.fieldConstants.redHubXPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getX());
            Logger.recordOutput("Vision/yPositionHubRelative", Constants.fieldConstants.redHubYPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getY());
            Logger.recordOutput("Vision/hubAngleFieldRelative",
                Math.atan2(
                    Constants.fieldConstants.redHubYPosM
                        - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getY(),
                    Constants.fieldConstants.redHubXPosM
                        - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getX()
                )  
                + Math.PI
            );
            Logger.recordOutput("Vision/hubDistanceFieldRelative", RobotContainer.vis.getHubDistanceFieldRelative());
            Logger.recordOutput("Vision/proportionalShooterSpeed", RobotContainer.m_ShooterS.getShooterProportionalControlSpeedRPM());
        }
      
        public static double getHubAngleFieldRelative() {
            Optional<Alliance> alliance = DriverStation.getAlliance();
            if (alliance.get().equals(Alliance.Blue)){
                xPositionHubRelative = Constants.fieldConstants.blueHubXPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getX();
                yPositionHubRelative = Constants.fieldConstants.blueHubYPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getY();
            } else {
                xPositionHubRelative = Constants.fieldConstants.redHubXPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getX();
                yPositionHubRelative = Constants.fieldConstants.redHubYPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getY();
            }
        return Math.atan2(yPositionHubRelative, xPositionHubRelative) + Math.PI;
        }

    public double getHubDistanceFieldRelative() {
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.get().equals(Alliance.Blue)){
            xPositionHubRelative = Constants.fieldConstants.blueHubXPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getX();
            yPositionHubRelative = Constants.fieldConstants.blueHubYPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getY();
        } else {
            xPositionHubRelative = Constants.fieldConstants.redHubXPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getX();
            yPositionHubRelative = Constants.fieldConstants.redHubYPosM - RobotContainer.m_SwerveS.poseEstimator.getEstimatedPosition().getY();
        }

        return Math.sqrt(Math.pow(xPositionHubRelative, 2) + Math.pow(yPositionHubRelative, 2));
    }

}

