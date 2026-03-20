package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    int[] validIDs = {5,8,9,10,4,3,2,11}; // ids to track
    boolean doRejectUpdate = false;

    PoseEstimate mt2;

    @SuppressWarnings("")
    public Vision(){
        
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

        // Switch to pipeline 0
        LimelightHelpers.setPipelineIndex("limelight", 0);

        LimelightHelpers.setLEDMode_ForceOff("limelight");

        // Set a custom crop window for improved performance (-1 to 1 for each value)
        LimelightHelpers.setCropWindow("limelight", -0.5, 0.5, -0.5, 0.5);


        // Change the camera pose relative to robot center (x forward, y left, z up, degrees)
        LimelightHelpers.setCameraPose_RobotSpace("limelight", 
            0.4216,    // Forward offset (meters)
            0.1905,    // Side offset (meters)
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
        LimelightHelpers.SetRobotOrientation("limelight", SwerveS.poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        
        // if our angular velocity is greater than 360 degrees per second, ignore vision updates
        if(Math.abs(RobotContainer.gyro.getAngularVelocityYDevice().getValueAsDouble()) > 360 && mt2 != null){
            doRejectUpdate = true;

        } else if(mt2.tagCount == 0) {
            doRejectUpdate = true;

        } else if(!doRejectUpdate) {
            
            SwerveS.poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            SwerveS.poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
        }

        Logger.recordOutput("Vision/tx", LimelightHelpers.getTX("limelight"));
        Logger.recordOutput("Vision/ty", LimelightHelpers.getTY("limelight"));
        Logger.recordOutput("Vision/ta", LimelightHelpers.getTA("limelight"));        
    }

    public double limelight_aim_proportional(){    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= SwerveS.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  public double limelight_range_proportional(){    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= SwerveS.kMaxSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }
    
}

