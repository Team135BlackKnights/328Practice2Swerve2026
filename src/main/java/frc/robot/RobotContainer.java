// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeRollerC;
import frc.robot.commands.MoveIntakeC;
import frc.robot.commands.ShooterC;
import frc.robot.commands.ShooterHoodC;
import frc.robot.commands.SwerveC;
import frc.robot.commands.XLock;
import frc.robot.subsystems.HoodAngleS;
import frc.robot.subsystems.IntakeRollerS;
import frc.robot.subsystems.MoveIntakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.SwerveS;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // Replace with CommandPS4Controller or CommandJoystick if needed
  public static final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  public static final CommandXboxController m_manipulatorController = 
      new CommandXboxController(OperatorConstants.kManipulatorControllerPort);
  
  private final static Pigeon2 gyro = new Pigeon2(Constants.PigeonConstants.pigeonID, "rio");
  
    public static final Trigger xDriverButton = m_driverController.x();
    public static final Trigger yDriverButton = m_driverController.y();
    public static final Trigger aDriverButton = m_driverController.a();
    public static final Trigger bDriverButton = m_driverController.b();
    public static final Trigger startDriverButton = m_driverController.start();
    public static final Trigger lDriverBumper = m_driverController.leftBumper();
    public static final Trigger rDriverBumper = m_driverController.rightBumper();
    public static final Trigger lDriverTrigger = m_driverController.leftTrigger();
    public static final Trigger rDriverTrigger = m_driverController.rightTrigger();
  
    public static final Trigger xManipulatorButton = m_manipulatorController.x();
    public static final Trigger yManipulatorButton = m_manipulatorController.y();
    public static final Trigger aManipulatorButton = m_manipulatorController.a();
    public static final Trigger bManipulatorButton = m_manipulatorController.b();
    public static final Trigger lManipulatorBumper = m_manipulatorController.leftBumper();
    public static final Trigger rManipulatorBumper = m_manipulatorController.rightBumper();
    public static final Trigger lManipulatorTrigger = m_manipulatorController.leftTrigger();
    public static final Trigger rManipulatorTrigger = m_manipulatorController.rightTrigger();
  
    SwerveS m_SwerveS = new SwerveS();
    MoveIntakeS m_MoveIntakeS = new MoveIntakeS();
    HoodAngleS m_HoodAngleS = new HoodAngleS ();
    ShooterS m_ShooterS = new ShooterS ();
    IntakeRollerS m_IntakeRollerS = new IntakeRollerS ();
  
    SwerveC m_SwerveC = new SwerveC(m_SwerveS);
    MoveIntakeC m_MoveIntakeDownC = new MoveIntakeC(m_MoveIntakeS, .25);//mess with p so it work(use the PID in HoodAngles) placeholding :)
    MoveIntakeC m_MoveIntakeUpC = new MoveIntakeC(m_MoveIntakeS, 0);
    XLock m_XLock = new XLock(m_SwerveS);
    ShooterHoodC m_ShooterHoodNegativeC = new ShooterHoodC(m_HoodAngleS, Constants.HoodConstants.hoodAngleVoltageNegative);
    ShooterHoodC m_ShooterHoodPositiveC = new ShooterHoodC(m_HoodAngleS, Constants.HoodConstants.hoodAngleVoltagePositive);
    ShooterHoodC m_ShooterHoodStopC = new ShooterHoodC(m_HoodAngleS, 0);
    ShooterC m_ShooterC = new ShooterC(m_ShooterS, Constants.ShooterConstants.shooter1Voltage, Constants.ShooterConstants.shooter2Voltage);
    ShooterC m_ShooterStopC = new ShooterC(m_ShooterS, 0, 0);
    IntakeRollerC m_IntakeRollerC = new IntakeRollerC(m_IntakeRollerS, Constants.IntakeRollerConstants.rollerVoltage);
    // RollerC m_RollerStopC = new RollerC(m_IntakeRollerS, 0); should be unnecessary bcz m_RollerC is on a toggleOnTrue
  
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
  
      m_SwerveS.setDefaultCommand(m_SwerveC);
      // Configure the trigger bindings
      configureBindings();
    }
  
    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
      // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
  
      // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
      // cancelling on release.
  
  
      //xButtonTrigger.whileTrue(m_IntakeC); this is how you do that
      xDriverButton.toggleOnTrue(m_XLock);
      aDriverButton.onTrue(new InstantCommand(() -> gyro.setYaw(0)));



      
      aManipulatorButton.toggleOnTrue(m_MoveIntakeDownC);
      aManipulatorButton.toggleOnFalse(m_MoveIntakeUpC);
      bManipulatorButton.toggleOnTrue(m_IntakeRollerC);    
      lManipulatorBumper.whileTrue(m_ShooterHoodNegativeC);
      rManipulatorBumper.whileTrue(m_ShooterHoodPositiveC);
      rManipulatorBumper.or(lManipulatorBumper).whileFalse(m_ShooterHoodStopC); //fancy logic(look up truth table for OR to understand)
      rManipulatorTrigger.whileTrue(m_ShooterC);
      rManipulatorTrigger.whileFalse(m_ShooterStopC);
  
    }
  
    // field oriented code. snowball's chance in flames it'll work
    
    // int invert = 1;
  
    public static ChassisSpeeds fieldOrientedDrive(double x, double y, double rot){
  
      /*  this is copied from SwerveC. i think it makes the robot go smoother idk
          double x = RobotContainer.m_driverController.getLeftX();
          double y = RobotContainer.m_driverController.getLeftY();
          double angle = Math.atan2(y,x);
          double magnitude = Math.hypot(x, y);
          double a = 0.6;
          magnitude = a*Math.pow(magnitude,5)+magnitude*(1-a);
          x = Math.cos(angle) * magnitude;
          y = Math.sin(angle) * magnitude;
  
          double rot = RobotContainer.m_driverController.getRightX();
          rot = a*Math.pow(rot,5)+rot*(1-a);
      */
  
    // The origin is always blue. When our alliance is red, X and Y need to be inverted
      Optional<Alliance> alliance = DriverStation.getAlliance();
    int invert = 1;
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      invert = -1;
    }
    // Create field relative ChassisSpeeds for controlling Swerve
    ChassisSpeeds cspeeds = new ChassisSpeeds(0,0,0); 
  
    cspeeds = ChassisSpeeds.fromFieldRelativeSpeeds(x * invert, y * invert, rot, gyro.getRotation2d());
    return cspeeds;
  }
}
