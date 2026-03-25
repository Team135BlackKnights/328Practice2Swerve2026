package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.LoggableTunedNumber;
import frc.robot.RobotContainer;
import frc.robot.Utils;

public class SwerveS extends SubsystemBase{
    static SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        SwerveConstants.moduleLocationFrontLeft,
        SwerveConstants.moduleLocationFrontRight,
        SwerveConstants.moduleLocationBackLeft, 
        SwerveConstants.moduleLocationBackRight
    );
    static CANBus bus = new CANBus("E13B8EB250374E5320202047380C10FF");
    //public static CANBus bus = new CANBus("canivore", "./logs/example.hoot");

    private final LoggableTunedNumber kP = new LoggableTunedNumber("Aim/kP",Constants.SwerveConstants.aimPID[0],true);
    private final LoggableTunedNumber kI = new LoggableTunedNumber("Aim/kI",Constants.SwerveConstants.aimPID[1],true);
    private final LoggableTunedNumber kD = new LoggableTunedNumber("Aim/kD",Constants.SwerveConstants.aimPID[2],true);
    

    static SwerveModule frontLeftModule = new SwerveModule(SwerveConstants.frontLeftTurnID, SwerveConstants.driveLeftInversion,SwerveConstants.frontLeftDriveID, SwerveConstants.frontLeftEncoderID, SwerveConstants.frontLeftOffsetRadians, bus);
    static SwerveModule frontRightModule = new SwerveModule(SwerveConstants.frontRightTurnID , SwerveConstants.driveRightInversion,SwerveConstants.frontRightDriveID, SwerveConstants.frontRightEncoderID, SwerveConstants.frontRightOffsetRadians, bus);
    static SwerveModule backLeftModule = new SwerveModule(SwerveConstants.backLeftTurnID, SwerveConstants.driveLeftInversion, SwerveConstants.backLeftDriveID, SwerveConstants.backLeftEncoderID, SwerveConstants.backLeftOffsetRadians, bus);
    static SwerveModule backRightModule = new SwerveModule(SwerveConstants.backRightTurnID, SwerveConstants.driveRightInversion, SwerveConstants.backRightDriveID, SwerveConstants.backRightEncoderID, SwerveConstants.backRightOffsetRadians, bus);
    static PIDController aimController =  new PIDController(SwerveConstants.aimPID[0], SwerveConstants.aimPID[1], SwerveConstants.aimPID[2]); 
    
    public static final double kMaxSpeed = 2; // max translational speed limelight is allowed to use
    public static final double kMaxAngularSpeed = 2; // max aim limelight is allowed to use

    public SwerveS(){
        aimController.enableContinuousInput(0, 2 * Math.PI);
    }

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
        m_kinematics, 
        RobotContainer.gyro.getRotation2d(),
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeftModule.getPositionMeters(), frontLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(frontRightModule.getPositionMeters(), frontRightModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backLeftModule.getPositionMeters(), backLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backRightModule.getPositionMeters(), backRightModule.getTurnPositionRotation2D())
        }, 
        m_pose
    );

    static Pose2d m_pose = new Pose2d(Units.inchesToMeters(510),Units.inchesToMeters(150), new Rotation2d(Math.toRadians(180)));
    
    public final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(
        m_kinematics, 
        RobotContainer.gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeftModule.getPositionMeters(), frontLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(frontRightModule.getPositionMeters(), frontRightModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backLeftModule.getPositionMeters(), backLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backRightModule.getPositionMeters(), backRightModule.getTurnPositionRotation2D())
        }, 
        m_pose);

    public void periodic(){

        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            aimController = new PIDController(kP.get(), kI.get(), kD.get());
            aimController.enableContinuousInput(0, 2 * Math.PI);
        }, kP, kI, kD);

        frontLeftModule.updateStatePID();
        frontRightModule.updateStatePID();
        backLeftModule.updateStatePID();
        backRightModule.updateStatePID();
        Logger.recordOutput("Swerve/Estimated X position", poseEstimator.getEstimatedPosition().getX());
        Logger.recordOutput("Swerve/Estimated Y position", poseEstimator.getEstimatedPosition().getY());
        // if(print > 0) {
        //     System.out.println("FR:" + frontLeftModule.getTurnPosition());
        //     System.out.println("FL:" + frontRightModule.getTurnPosition());
        //     System.out.println("BL:" + backLeftModule.getTurnPosition());
        //     System.out.println("BR:" + backRightModule.getTurnPosition());
        //     System.out.println();
        //     print = 0;
        // } else print ++;
        Logger.recordOutput("Swerve/frontLeftModulePosition", frontLeftModule.getPositionMeters());
        Logger.recordOutput("Swerve/frontRightModulePosition", frontRightModule.getPositionMeters());
        Logger.recordOutput("Swerve/backLeftModulePosition", backLeftModule.getPositionMeters());
        Logger.recordOutput("Swerve/backRightModulePosition", backRightModule.getPositionMeters());

        
    }

    public void updatePose(){
        var gyroAngle = RobotContainer.gyro.getRotation2d();
        // Update the pose
        m_pose = m_odometry.update(gyroAngle,
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeftModule.getPositionMeters(), frontLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(frontRightModule.getPositionMeters(), frontRightModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backLeftModule.getPositionMeters(), backLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backRightModule.getPositionMeters(), backRightModule.getTurnPositionRotation2D())
        });

    }

    public void updatePoseEsitmator(){
        poseEstimator.update(RobotContainer.gyro.getRotation2d(), 
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeftModule.getPositionMeters(), frontLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(frontRightModule.getPositionMeters(), frontRightModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backLeftModule.getPositionMeters(), backLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backRightModule.getPositionMeters(), backRightModule.getTurnPositionRotation2D())
            }
        );
        //adding vision measurement is done in Vision
        //poseEstimator.addVisionMeasurement(LimelightHelpers.getBotPose2d("limelight"), Timer.getFPGATimestamp());
    }

    public ChassisSpeeds getState(){
        return m_kinematics.toChassisSpeeds(
            new SwerveModuleState(frontLeftModule.getDriveSpeedMetersPerSecond(), frontLeftModule.getTurnPositionRotation2D()),
            new SwerveModuleState(frontRightModule.getDriveSpeedMetersPerSecond(), frontRightModule.getTurnPositionRotation2D()),
            new SwerveModuleState(backLeftModule.getDriveSpeedMetersPerSecond(), backLeftModule.getTurnPositionRotation2D()),
            new SwerveModuleState(backRightModule.getDriveSpeedMetersPerSecond(), backRightModule.getTurnPositionRotation2D())
        );
    }

    public void setSpeed(double xSpeed, double ySpeed, double rotSpeed){
        ChassisSpeeds speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);
        SwerveModuleState frontLeft = moduleStates[0];
        SwerveModuleState frontRight = moduleStates[1];
        SwerveModuleState backLeft = moduleStates[2];
        SwerveModuleState backRight = moduleStates[3];

        // System.out.println("FR:" + frontLeft);
        // System.out.println("FL:" + frontRight);
        // System.out.println("BL:" + backLeft);
        // System.out.println("BR:" + backRight);
        
        frontLeftModule.setDesiredModuleState(frontLeft);
        frontRightModule.setDesiredModuleState(frontRight);
        backLeftModule.setDesiredModuleState(backLeft);
        backRightModule.setDesiredModuleState(backRight);

    }

    public Pose2d getPose(){
        return poseEstimator.getEstimatedPosition();
    }
    public void setSpeedFromState(ChassisSpeeds state){
        setSpeed(state.vxMetersPerSecond, state.vyMetersPerSecond, state.omegaRadiansPerSecond);
    }

    public void setModuleStates(SwerveModuleState frontLeft, SwerveModuleState frontRight, SwerveModuleState backLeft, SwerveModuleState backRight){
        frontLeftModule.setDesiredModuleState(frontLeft);
        frontRightModule.setDesiredModuleState(frontRight);
        backLeftModule.setDesiredModuleState(backLeft);
        backRightModule.setDesiredModuleState(backRight);
    }
    
    // public SwerveModuleState optimizeWithCosineCompensation(
    //         SwerveModuleState desiredState,
    //         Rotation2d currentAngle) {
    //     // Standard WPILib optimization (handles 180° flips)
    //     SwerveModuleState optimized =
    //             SwerveModuleState.optimize(desiredState, currentAngle);
    //     // Calculate angle error
    //     Rotation2d angleError =
    //             optimized.angle.minus(currentAngle);
    //     // Apply cosine compensation
    //  optimized.speedMetersPerSecond *= angleError.getCos();

    //     return optimized;
    // }

    public static double getAimToPointSpeedRadians(double fieldRelativeAngle){
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.get().equals(Alliance.Blue)){
            fieldRelativeAngle += Math.PI;
        }
        fieldRelativeAngle = fieldRelativeAngle % (2*Math.PI);
        double gyroAngle = Utils.mod(RobotContainer.gyro.getYaw().getValueAsDouble() / 180 * Math.PI, 2 * Math.PI);
        if (Math.abs(fieldRelativeAngle - gyroAngle) < Math.toRadians(2.5)){
            return 0;
        }
        return aimController.calculate(gyroAngle, fieldRelativeAngle);
    }


    public void swervePathPlanner() {
        // All other subsystem initialization
        // ...

        

        // Load the RobotConfig from the GUI settings. You should probably
        // store this in your Constants file
        RobotConfig config;
        try{
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            config = null;
            // config = new RobotConfig(Constants.robotMassKG, 0, new ModuleConfig(Constants.SwerveConstants.wheelRadius, Constants.SwerveConstants.maxLinearSpeedMPS, 1000, 2, 12, 1), Constants.SwerveConstants.moduleLocationFrontLeft,Constants.SwerveConstants.moduleLocationFrontRight,Constants.SwerveConstants.moduleLocationBackLeft,Constants.SwerveConstants.moduleLocationBackRight);
        }

        // Configure AutoBuilder last
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            (newPose)->{m_pose = newPose;
            RobotContainer.gyro.setYaw(0);//newPose.getRotation().getDegrees());
            m_odometry.resetPose(newPose);
            }, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getState, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, ff) -> setSpeedFromState(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                new PIDConstants(1.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.01) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
            },
            this // Reference to this subsystem to set requirements
        );


  }


  public double getflTurnVoltage(){
    return frontLeftModule.turnVoltage;
  }

  public double getflTurnPos(){
    return frontLeftModule.getPositionMeters();
  }

  public void setflTurnVoltage(double voltage){
    frontLeftModule.setTurnVoltage(voltage);
  }
}



