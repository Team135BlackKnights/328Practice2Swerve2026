// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.DriverStation;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;

import frc.robot.commands.SwerveC;
import frc.robot.commands.XLock;
import frc.robot.subsystems.HangS;
import frc.robot.subsystems.HoodAngleS;
import frc.robot.subsystems.IndexerS;
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
  
  private final SendableChooser<Command> autoChooser;

  double hoodSetpoint = 0;

  static CANBus bus = new CANBus("E13B8EB250374E5320202047380C10FF");

  public static double linearSpeedMultiplier = 7.5;
  public static double radianSpeedMultiplier = 10;
  public final static Pigeon2 gyro = new Pigeon2(Constants.PigeonConstants.pigeonID, bus);
  
    public static final Trigger xDriverButton = m_driverController.x();
    public static final Trigger yDriverButton = m_driverController.y();
    public static final Trigger aDriverButton = m_driverController.a();
    public static final Trigger bDriverButton = m_driverController.b();
    public static final Trigger startDriverButton = m_driverController.start();
    public static final Trigger lDriverBumper = m_driverController.leftBumper();
    public static final Trigger rDriverBumper = m_driverController.rightBumper();
    public static final Trigger lDriverTrigger = m_driverController.leftTrigger();
    public static final Trigger rDriverTrigger = m_driverController.rightTrigger();
    public static final Trigger lDriverStickButton = m_driverController.leftStick();

    public static final Trigger dpadDriverRight = new Trigger(() -> {
      double pov = m_driverController.getHID().getPOV();
      if (pov > 45 && pov < 135){
        System.out.println("driver right dpad");
        return true;
      } 
      else{
        return false;
      }
    });

    public static final Trigger dpadDriverUp = new Trigger(() -> {
      double pov = m_driverController.getHID().getPOV();
      if (pov > 335 || pov < 45){
        System.out.println("driver up dpad");
        return true;
      } 
      else{
        return false;
      }
    });

    public static final Trigger dpadDriverDown = new Trigger(() -> {
      double pov = m_driverController.getHID().getPOV();
      if (pov > 135 && pov < 240){
        System.out.println("driver down dpad");
        return true;
      } 
      else{
        return false;
      }
    });

    public static final Trigger dpadDriverLeft = new Trigger(() -> {
      double pov = m_driverController.getHID().getPOV();
      if (pov > 240 && pov < 315){
        System.out.println("driver left dpad");
        return true;
      } 
      else{
        return false;
      }
    });


    public static final Trigger dpadManipRight = new Trigger(() -> {
      double pov = m_manipulatorController.getHID().getPOV();
      if (pov > 45 && pov < 135){
        System.out.println("manip right dpad");
        return true;
      } 
      else{
        return false;
      }
    });

    public static final Trigger dpadManipUp = new Trigger(() -> {
      double pov = m_manipulatorController.getHID().getPOV();
      if (pov > 335 || pov < 45){
        System.out.println("manip up dpad");
        return true;
      } 
      else{
        return false;
      }
    });

    public static final Trigger dpadManipDown = new Trigger(() -> {
      double pov = m_manipulatorController.getHID().getPOV();
      if (pov > 135 && pov < 240){
        System.out.println("manip down dpad");
        return true;
      } 
      else{
        return false;
      }
    });

    public static final Trigger dpadManipLeft = new Trigger(() -> {
      double pov = m_manipulatorController.getHID().getPOV();
      if (pov > 240 && pov < 315){
        System.out.println("manip left dpad");
        return true;
      } 
      else{
        return false;
      }
    });

    public static final Trigger xManipulatorButton = m_manipulatorController.x();
    public static final Trigger yManipulatorButton = m_manipulatorController.y();
    public static final Trigger aManipulatorButton = m_manipulatorController.a();
    public static final Trigger bManipulatorButton = m_manipulatorController.b();
    public static final Trigger lManipulatorBumper = m_manipulatorController.leftBumper();
    public static final Trigger rManipulatorBumper = m_manipulatorController.rightBumper();
    public static final Trigger lManipulatorTrigger = m_manipulatorController.leftTrigger();
    public static final Trigger rManipulatorTrigger = m_manipulatorController.rightTrigger();
    
    

    static SwerveS m_SwerveS = new SwerveS();
    static MoveIntakeS m_MoveIntakeS = new MoveIntakeS();
    static HoodAngleS m_HoodAngleS = new HoodAngleS ();
    static ShooterS m_ShooterS = new ShooterS ();
    static IntakeRollerS m_IntakeRollerS = new IntakeRollerS ();
    static HangS m_HangS = new HangS();
    static IndexerS m_IndexerS = new IndexerS();
  
    SwerveC m_SwerveC = new SwerveC(m_SwerveS);
    // MoveIntakeC m_MoveIntakeDownC = new MoveIntakeC(m_MoveIntakeS, .25);//mess with p so it work(use the PID in HoodAngles) placeholding :)
    // MoveIntakeC m_MoveIntakeUpC = new MoveIntakeC(m_MoveIntakeS, 0);
    XLock m_XLock = new XLock(m_SwerveS);
    // ShooterHoodC m_ShooterHoodNegativeC = new ShooterHoodC(m_HoodAngleS, Constants.HoodConstants.hoodAngleVoltageNegative);
    // ShooterHoodC m_ShooterHoodPositiveC = new ShooterHoodC(m_HoodAngleS, Constants.HoodConstants.hoodAngleVoltagePositive);
    // ShooterHoodC m_ShooterHoodStopC = new ShooterHoodC(m_HoodAngleS, 0);

    // ShooterC m_ShooterC = new ShooterC(m_ShooterS, Constants.ShooterConstants.shooter1Voltage, Constants.ShooterConstants.shooter2voltage);
    // IntakeRollerC m_IntakeRollerC = new IntakeRollerC(m_IntakeRollerS, Constants.IntakeRollerConstants.rollerVoltage);
    // IntakeRollerC m_IntakeRollerOffC = new IntakeRollerC(m_IntakeRollerS, 0);
    // HangC m_HangC = new HangC(m_HangS, Constants.IntakeRollerConstants.rollerVoltage);
    // IndexerC m_IndexerC = new IndexerC(m_IndexerS, Constants.IndexerConstants.indexerVoltage);
    // RollerC m_RollerStopC = new RollerC(m_IntakeRollerS, 0); should be unnecessary bcz m_RollerC is on a toggleOnTrue
  
    //ParallelCommandGroup shootAndIndex = new ParallelCommandGroup(Commands.run(() -> m_ShooterS.fire(Constants.ShooterConstants.shooter1Voltage, Constants.ShooterConstants.shooter2voltage),m_ShooterS).finallyDo(() -> m_ShooterS.fire(-2, -2)), Commands.run(() -> m_IndexerS.setVoltage(Constants.IndexerConstants.indexerVoltage), m_IndexerS).finallyDo(() -> m_IndexerS.setVoltage(0)));

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
       // ...

    // Build an auto chooser. This will use Commands.none() as the default option.
    m_SwerveS.swervePathPlanner();
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);
      
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
      
      // yDriverButton is bound to limelight (see Robot.java)
      xDriverButton.toggleOnTrue(m_XLock);
      bDriverButton.onTrue(new InstantCommand(() -> gyro.setYaw(0)));
      rDriverBumper.toggleOnTrue(Commands.run(() -> m_HangS.hangPower(Constants.HangConstants.hangVoltage), m_HangS).finallyDo(() -> m_HangS.hangPower(0)));
      aDriverButton.onTrue(new InstantCommand(() -> RobotContainer.linearSpeedMultiplier = 10 ).finallyDo(() -> RobotContainer.linearSpeedMultiplier = 7.5));

      bManipulatorButton.whileTrue((Commands.run(() -> m_MoveIntakeS.moveTo(Constants.IntakeConstants.downPositionSetpoint),m_MoveIntakeS)).finallyDo(() -> m_MoveIntakeS.setVoltage(0)));
      yManipulatorButton.whileTrue((Commands.run(() -> m_MoveIntakeS.moveTo(Constants.IntakeConstants.upPositionSetpoint),m_MoveIntakeS)).finallyDo(() -> m_MoveIntakeS.setVoltage(0)));
      
      xManipulatorButton.whileTrue(Commands.run(() -> m_IntakeRollerS.rollerSpeed(Constants.IntakeRollerConstants.rollerVoltage),m_IntakeRollerS).finallyDo(() -> m_IntakeRollerS.rollerSpeed(0)));
      dpadManipDown.whileTrue(Commands.run(() -> m_IntakeRollerS.rollerSpeed(-1*Constants.IntakeRollerConstants.rollerVoltage),m_IntakeRollerS).finallyDo(() -> m_IntakeRollerS.rollerSpeed(0)));

      lManipulatorBumper.onTrue(new InstantCommand(() -> {
        hoodSetpoint = MathUtil.clamp(hoodSetpoint += 0.05, 0.0, 0.25);
        Commands.run(() -> m_HoodAngleS.moveHood(hoodSetpoint));
      }));

      rManipulatorBumper.onTrue(new InstantCommand(() -> {
        hoodSetpoint = MathUtil.clamp(hoodSetpoint -= 0.05, 0.0, 0.25);
        Commands.run(() -> m_HoodAngleS.moveHood(hoodSetpoint));
      }));



      //aManipulatorButton.whileTrue(Commands.run(() -> m_ShooterS.fire(Constants.ShooterConstants.shooter1Voltage, Constants.ShooterConstants.shooter2voltage),m_ShooterS).finallyDo(() -> m_ShooterS.fire(0, 0)));
      //aManipulatorButton.whileTrue(Commands.run(() -> m_IndexerS.setVoltage(Constants.IndexerConstants.indexerVoltage), m_IndexerS).finallyDo(() -> m_IndexerS.setVoltage(0))); // to change it to both on 
      
      // aDriverButton.toggleOnTrue(new InstantCommand(() -> System.out.println("a manip button")));
      // bDriverButton.toggleOnTrue(new InstantCommand(() -> System.out.println("b manip button")));
      // xDriverButton.toggleOnTrue(new InstantCommand(() -> System.out.println("x manip button")));
      // yDriverButton.toggleOnTrue(new InstantCommand(() -> System.out.println("y manip button"))); 
      // rManipulatorBumper.or(lManipulatorBumper).whileFalse(m_ShooterHoodStopC);

      aManipulatorButton.whileTrue(Commands.run(() -> m_ShooterS.fire(Constants.ShooterConstants.shooter1Voltage, Constants.ShooterConstants.shooter2voltage),m_ShooterS).finallyDo(() -> m_ShooterS.fire(0, -2)).raceWith(Commands.run(() -> m_IndexerS.setVoltage(Constants.IndexerConstants.indexerVoltage), m_IndexerS).finallyDo(() -> m_IndexerS.setVoltage(0))));

      rManipulatorTrigger.onTrue(Commands.run(() -> m_ShooterS.fire(0, 0),m_ShooterS)); //sets the flywheel to 0 speed, shoot button will need pressing again
      
    }
      
  public static ChassisSpeeds fieldOrientedDrive(double x, double y, double rot){

    Optional<Alliance> invert = DriverStation.getAlliance();
    Rotation2d gyroDirection = gyro.getRotation2d();
    if (invert.get().equals(Alliance.Red)){
      gyroDirection = gyroDirection.plus(new Rotation2d(Math.PI));
    }
    return ChassisSpeeds.fromFieldRelativeSpeeds(x,y, rot, gyroDirection);

  }

  public Command getAutonomousCommand() {
    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path
    //return new PathPlannerAuto("Example Auto");
    return autoChooser.getSelected();
  }

  
}
