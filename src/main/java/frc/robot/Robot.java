// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;
import org.littletonrobotics.urcl.URCL;

import com.revrobotics.util.StatusLogger;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
//import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.math.MathUtil;
//import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.PixelFormat;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
//import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.SwerveS;

/**
 * The methods in this class are called automatically corresponding to each mode, as described in
 * the TimedRobot documentation. If you change the name of this class or the package after creating
 * this project, you must also update the Main.java file in the project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;
  //  private final Swerve m_swerve = new Swerve(); used in the last setSpeed by limelight, irrevelent

  private final RobotContainer m_robotContainer;
  public static double intakeSetpoint = 0;
  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter m_xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);
  //private boolean doRejectUpdate = false;

  UsbCamera intakeCamera;
  UsbCamera frontCamera;
  //UsbCamera camera3;
  VideoSink server;
  CvSource output;

  DataLog log;
  DoubleArrayLogEntry motorLog;

  @Override
  public void robotInit(){
    DataLogManager.start();
    log = DataLogManager.getLog();
    DriverStation.startDataLog(log);
    motorLog = new DoubleArrayLogEntry(log, "/Motor/PositionVoltage");
  }

  @Override
  public void autonomousPeriodic() {
    System.out.println("Auto Command: " + m_autonomousCommand.getName());
  }

  @Override
  public void teleopPeriodic() {

    drive(true);
    Logger.recordOutput("Battery Voltage", RobotController.getBatteryVoltage());
    Logger.recordOutput("Gyro X Accel", RobotContainer.gyro.getAccelerationX().getValueAsDouble());
    Logger.recordOutput("Gyro Y Accel", RobotContainer.gyro.getAccelerationY().getValueAsDouble());
    Logger.recordOutput("Gyro Z Accel", RobotContainer.gyro.getAccelerationZ().getValueAsDouble());

    RobotContainer.m_MoveIntakeS.moveTo(intakeSetpoint);
    
    if (RobotContainer.rDriverBumper.getAsBoolean()){
      System.out.println("intake camera feed");
      server.setSource(intakeCamera);
    }else if (RobotContainer.lDriverBumper.getAsBoolean()){
      System.out.println("front camera feed");
      server.setSource(frontCamera);
    }

    if (RobotContainer.m_manipulatorController.getRightTriggerAxis() > 0){
      System.out.println("firing");
      RobotContainer.m_ShooterS.fire(Constants.ShooterConstants.shooter1Voltage*RobotContainer.m_manipulatorController.getRightTriggerAxis(),Constants.ShooterConstants.flywheelRPM*RobotContainer.m_manipulatorController.getRightTriggerAxis());
      RobotContainer.m_IndexerS.setVoltage(Constants.IndexerConstants.indexerVoltage);
    } else if (RobotContainer.aManipulatorButton.getAsBoolean()){
      System.out.println("reversing subsystems");
      RobotContainer.m_IndexerS.setVoltage(-1*Constants.IndexerConstants.indexerVoltage);
      RobotContainer.m_IntakeRollerS.rollerSpeed(-1*Constants.IntakeRollerConstants.rollerVoltage);
      RobotContainer.m_ShooterS.fire(-1*Constants.ShooterConstants.shooter1Voltage, -1*Constants.ShooterConstants.flywheelRPM);
    }else if (RobotContainer.lManipulatorTrigger.getAsBoolean()){
      RobotContainer.m_ShooterS.stop();
    }else if (RobotContainer.yManipulatorButton.getAsBoolean()){
      RobotContainer.m_ShooterS.fire(0,Constants.ShooterConstants.flywheelRPM);
    }else
    {
      RobotContainer.m_IndexerS.setVoltage(0);
      RobotContainer.m_IntakeRollerS.rollerSpeed(0);
      RobotContainer.m_ShooterS.idle(-1.3);
    }

    
  }

  // simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  
  private void drive(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed =
        -m_xspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftY(), 0.02))
            * SwerveS.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed =
        -m_yspeedLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_driverController.getLeftX(), 0.02))
            * SwerveS.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
  
    // the right by default.
    var rot =
        -m_rotLimiter.calculate(MathUtil.applyDeadband(RobotContainer.m_driverController.getRightX(), 0.02))
            * SwerveS.kMaxAngularSpeed;

  
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  public Robot() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    Logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    Logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    Logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    Logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    Logger.recordMetadata(
        "GitDirty",
        switch (BuildConstants.DIRTY) {
          case 0 -> "All changes committed";
          case 1 -> "Uncommitted changes";
          default -> "Unknown";
        });


        Logger.addDataReceiver(new WPILOGWriter());
        Logger.addDataReceiver(new NT4Publisher());

    // Initialize URCL
    Logger.registerURCL(URCL.startExternal());
    StatusLogger.disableAutoLogging(); // Disable REVLib's built-in logging

    // Start AdvantageKit logger
    Logger.start();

    m_robotContainer = new RobotContainer();

    /**
    * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
    * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
    * to the dashboard. Just add this to the robot class constructor.
    */
    intakeCamera = CameraServer.startAutomaticCapture(0);
    frontCamera = CameraServer.startAutomaticCapture(1);
    intakeCamera.setVideoMode(PixelFormat.kMJPEG, 420, 380, 30);
    server = CameraServer.getServer();
    server.setSource(intakeCamera);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    RobotContainer.m_SwerveS.updatePose();
    RobotContainer.m_SwerveS.updatePoseEsitmator();
    CommandScheduler.getInstance().run();
    motorLog.append(new double[] {m_robotContainer.getflWheelPos(), m_robotContainer.getflWheelVotage()});
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

 // @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    //schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
    for (int i = 0;  i == 50; i++){
    System.out.println("Running Auto: " + m_autonomousCommand.getName());
    }
    CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


}
