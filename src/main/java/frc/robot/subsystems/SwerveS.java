package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;

public class SwerveS extends SubsystemBase{
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(SwerveConstants.moduleLocationFrontLeft,SwerveConstants.moduleLocationFrontRight,
    SwerveConstants.moduleLocationBackLeft, SwerveConstants.moduleLocationBackRight);
    CANBus bus = new CANBus("E13B8EB250374E5320202047380C10FF");
    //public static CANBus bus = new CANBus("canivore", "./logs/example.hoot");
    SwerveModule frontLeftModule = new SwerveModule(SwerveConstants.frontLeftTurnID, SwerveConstants.frontLeftDriveID, SwerveConstants.frontLeftEncoderID, SwerveConstants.frontLeftOffsetRadians, bus);
    SwerveModule frontRightModule = new SwerveModule(SwerveConstants.frontRightTurnID, SwerveConstants.frontRightDriveID, SwerveConstants.frontRightEncoderID, SwerveConstants.frontRightOffsetRadians, bus);
    SwerveModule backLeftModule = new SwerveModule(SwerveConstants.backLeftTurnID, SwerveConstants.backLeftDriveID, SwerveConstants.backLeftEncoderID, SwerveConstants.backLeftOffsetRadians, bus);
    SwerveModule backRightModule = new SwerveModule(SwerveConstants.backRightTurnID, SwerveConstants.backRightDriveID, SwerveConstants.backRightEncoderID, SwerveConstants.backRightOffsetRadians, bus);
    
    public static final double kMaxSpeed = 3.0;
    public static final double kMaxAngularSpeed = Math.PI;

    SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
    m_kinematics, RobotContainer.gyro.getRotation2d(),
    new SwerveModulePosition[] {
        new SwerveModulePosition(frontLeftModule.getPosition(), frontLeftModule.getTurnPositionRotation2D()),
        new SwerveModulePosition(frontRightModule.getPosition(), frontRightModule.getTurnPositionRotation2D()),
        new SwerveModulePosition(backLeftModule.getPosition(), backLeftModule.getTurnPositionRotation2D()),
        new SwerveModulePosition(backRightModule.getPosition(), backRightModule.getTurnPositionRotation2D())
  }, new Pose2d(5.0, 13.5, new Rotation2d()));//todonot

    Pose2d m_pose = new Pose2d(3.572,2.682, new Rotation2d(Math.PI));
      
    public void periodic(){
        frontLeftModule.updateStatePID();
        frontRightModule.updateStatePID();
        backLeftModule.updateStatePID();
        backRightModule.updateStatePID();
        // System.out.println("FR:" + frontLeftModule.getTurnPosition());
        // System.out.println("FL:" + frontRightModule.getTurnPosition());
        // System.out.println("BL:" + backLeftModule.getTurnPosition());
        // System.out.println("BR:" + backRightModule.getTurnPosition());
        var gyroAngle = RobotContainer.gyro.getRotation2d();
        // Update the pose
        m_pose = m_odometry.update(gyroAngle,
        new SwerveModulePosition[] {
            new SwerveModulePosition(frontLeftModule.getPosition(), frontLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(frontRightModule.getPosition(), frontRightModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backLeftModule.getPosition(), backLeftModule.getTurnPositionRotation2D()),
            new SwerveModulePosition(backRightModule.getPosition(), backRightModule.getTurnPositionRotation2D())
        });

    }

    public ChassisSpeeds getState(){
        return m_kinematics.toChassisSpeeds(
            new SwerveModuleState(frontLeftModule.getDriveSpeed(), frontLeftModule.getTurnPositionRotation2D()),
            new SwerveModuleState(frontRightModule.getDriveSpeed(), frontRightModule.getTurnPositionRotation2D()),
            new SwerveModuleState(backLeftModule.getDriveSpeed(), backLeftModule.getTurnPositionRotation2D()),
            new SwerveModuleState(backRightModule.getDriveSpeed(), backRightModule.getTurnPositionRotation2D())
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
        return m_pose;
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
                ()->(m_pose), // Robot pose supplier
                (newPose)->{m_pose = newPose;}, // Method to reset odometry (will be called if your auto has a starting pose)
                this::getState, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                (speeds) -> setSpeedFromState(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                ),
                config, // The robot configuration
                () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return false;
                },
                this // Reference to this subsystem to set requirements
        );


  }

}



