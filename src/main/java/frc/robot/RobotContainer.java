// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;



import static edu.wpi.first.units.Units.Micro;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.HangC;
import frc.robot.commands.IndexerC;
import frc.robot.commands.IntakeRollerC;
import frc.robot.commands.MoveIntakeC;
import frc.robot.commands.ShooterC;
import frc.robot.commands.ShooterHoodC;
import frc.robot.commands.SwerveC;
import frc.robot.commands.XLock;
import frc.robot.commands.Autos.ShootAutoNoSwerve;
import frc.robot.commands.Autos.intakeA;
import frc.robot.subsystems.HangS;
import frc.robot.subsystems.HoodAngleS;
import frc.robot.subsystems.IndexerS;
import frc.robot.subsystems.IntakeRollerS;
import frc.robot.subsystems.MoveIntakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.SwerveS;
import frc.robot.subsystems.Vision;

import com.pathplanner.lib.auto.NamedCommands;


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
  
    //8.5 linear / 10 radian
    public static double linearSpeedMultiplier = 6;
    public static double radianSpeedMultiplier = -10;
  
    public final static Pigeon2 gyro = new Pigeon2(Constants.PigeonConstants.pigeonID, "E13B8EB250374E5320202047380C10FF");
    
      public static final Trigger xDriverButton = m_driverController.x();
      public static final Trigger yDriverButton = m_driverController.y();
      public static final Trigger aDriverButton = m_driverController.a();
      public static final Trigger bDriverButton = m_driverController.b();
      public static final Trigger startDriverButton = m_driverController.start();
      public static final Trigger lDriverBumper = m_driverController.leftBumper();
      public static final Trigger rDriverBumper = m_driverController.rightBumper();
      public static final Trigger lDriverTrigger = m_driverController.leftTrigger();
      public static final Trigger rDriverTrigger = m_driverController.rightTrigger();
      public static final Trigger rStickDriverButton = m_driverController.rightStick();
      public static final Trigger lStickDriverButton = m_driverController.leftStick();


      public static final Trigger dpadDriverRight = new Trigger(() -> {
        double pov = m_driverController.getHID().getPOV();
        if (pov > 45 && pov < 135){
          //System.out.println("driver right dpad");
          return true;
        } 
        else{
          return false;
        }
      });
  
      public static final Trigger dpadDriverUp = new Trigger(() -> {
        double pov = m_driverController.getHID().getPOV();
        if (pov > 335 || pov < 45){
          //System.out.println("driver up dpad");
          return true;
        } 
        else{
          return false;
        }
      });
  
      public static final Trigger dpadDriverDown = new Trigger(() -> {
        double pov = m_driverController.getHID().getPOV();
        if (pov > 135 && pov < 240){
          //System.out.println("driver down dpad");
          return true;
        } 
        else{
          return false;
        }
      });
  
      public static final Trigger dpadDriverLeft = new Trigger(() -> {
        double pov = m_driverController.getHID().getPOV();
        if (pov > 240 && pov < 315){
          //System.out.println("driver left dpad");
          return true;
        } 
        else{
          return false;
        }
      });
  
  
      public static final Trigger dpadManipRight = new Trigger(() -> {
        double pov = m_manipulatorController.getHID().getPOV();
        if (pov > 45 && pov < 135){
          //System.out.println("manip right dpad");
          return true;
        } 
        else{
          return false;
        }
      });
  
      public static final Trigger dpadManipUp = new Trigger(() -> {
        double pov = m_manipulatorController.getHID().getPOV();
        if (pov > 335 || pov < 45){
          //System.out.println("manip up dpad");
          return true;
        } 
        else{
          return false;
        }
      });
  
      public static final Trigger dpadManipDown = new Trigger(() -> {
        double pov = m_manipulatorController.getHID().getPOV();
        if (pov > 135 && pov < 240){
          //System.out.println("manip down dpad");
          return true;
        } 
        else{
          return false;
        }
      });
  
      public static final Trigger dpadManipLeft = new Trigger(() -> {
        double pov = m_manipulatorController.getHID().getPOV();
        if (pov > 240 && pov < 315){
          //System.out.println("manip left dpad");
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
      public static final Trigger leftStickManipulatorButton = m_manipulatorController.leftStick();
      

      public static final SwerveS m_SwerveS = new SwerveS();
      public static final MoveIntakeS m_MoveIntakeS = new MoveIntakeS();
      public static final HoodAngleS m_HoodAngleS = new HoodAngleS ();
      public static final ShooterS m_ShooterS = new ShooterS ();
      public static final IntakeRollerS m_IntakeRollerS = new IntakeRollerS ();
      public static final HangS m_HangS = new HangS();
      public static final IndexerS m_IndexerS = new IndexerS();

      
      //public static final DriveMeters driveturn = new DriveMeters(m_SwerveS, m_ShooterS, m_IndexerS, 3, 45);
    
      SwerveC m_SwerveC = new SwerveC(m_SwerveS);
      XLock m_XLock = new XLock(m_SwerveS);
      ShootAutoNoSwerve shootauto = new ShootAutoNoSwerve(m_IndexerS, m_ShooterS, m_MoveIntakeS, m_SwerveS);

      
      /** The container for the robot. Contains subsystems, OI devices, and commands. */
      public RobotContainer() {
        //blame lee if no work
      NamedCommands.registerCommand("fire", new InstantCommand(() -> m_ShooterS.fire(Constants.ShooterConstants.shooter1Voltage, Constants.ShooterConstants.flywheelRPM), m_ShooterS).finallyDo(() -> m_ShooterS.fire(0,0)));
      NamedCommands.registerCommand("intake", new intakeA(m_MoveIntakeS, m_IntakeRollerS));
      // Build an auto chooser. This will use Commands.none() as the default option.
      m_SwerveS.swervePathPlanner();   
      autoChooser = AutoBuilder.buildAutoChooser();     
      autoChooser.setDefaultOption("Do nothing", Commands.none());
      //what this *should* do is shoot and index for 6 seconds. should literally not move swerve at all
      autoChooser.addOption("shootauto", 
          new SequentialCommandGroup(
            Commands.run(
              () -> m_ShooterS.idle(-1.3), m_ShooterS
            )
            .alongWith(
              Commands.run(() -> m_SwerveS.setSpeed(0, 0, 0), m_SwerveS)
            )
            .withDeadline(
              Commands.waitTime(Seconds.of(5))
            ),
            shootauto.withDeadline(
              Commands.waitTime(Seconds.of(6))
            )
          )
      );
      
      // Another option that allows you to specify the default auto by its name
  
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
        
        xDriverButton.toggleOnTrue(m_XLock);
        bDriverButton.onTrue(new InstantCommand(() -> gyro.setYaw(0)));
        //y driver is reserved for hang
        aDriverButton.onTrue(new InstantCommand(() -> RobotContainer.linearSpeedMultiplier = 10 ).finallyDo(() -> RobotContainer.linearSpeedMultiplier = 7.5));
        yDriverButton.onTrue(new InstantCommand(() -> m_SwerveC.inverted = !m_SwerveC.inverted));

        lManipulatorBumper.whileTrue(new InstantCommand(() -> Robot.intakeSetpoint = Constants.IntakeConstants.upPositionSetpoint));
        rManipulatorBumper.whileTrue(new InstantCommand(() -> Robot.intakeSetpoint = Constants.IntakeConstants.downPositionSetpoint));
        leftStickManipulatorButton.onTrue(Commands.runOnce(() -> m_MoveIntakeS.zero()));
        xManipulatorButton.whileTrue(Commands.run(() -> m_IntakeRollerS.rollerSpeed(Constants.IntakeRollerConstants.rollerVoltage),m_IntakeRollerS).finallyDo(() -> m_IntakeRollerS.rollerSpeed(0)));
        rDriverBumper.whileTrue(Commands.run(() -> m_SwerveS.setflTurnVoltage(4 * m_driverController.getLeftTriggerAxis()), m_SwerveS));
        lDriverBumper.whileTrue(Commands.run(() -> m_SwerveS.setflTurnVoltage(-4 * m_driverController.getRightTriggerAxis()), m_SwerveS));

        //reverse things
        //this has been moved to robot.java
        //aManipulatorButton.whileTrue(Commands.run(() -> m_IndexerS.setVoltage(-1*Constants.IndexerConstants.indexerVoltage), m_IndexerS).finallyDo(() -> m_IndexerS.setVoltage(0)));
        //aManipulatorButton.whileTrue(Commands.run(() -> m_IntakeRollerS.rollerSpeed(-1*Constants.IntakeRollerConstants.rollerVoltage), m_IntakeRollerS).finallyDo(() -> m_IndexerS.setVoltage(0)));
        //aManipulatorButton.whileTrue(Commands.run(() -> m_ShooterS.fire(-1*Constants.ShooterConstants.shooter1Voltage, -1*Constants.ShooterConstants.shooter2voltage)));
        //set speed with triggers (in Robot.java)
        //TODO add a comment with that code for doing it off a trigger
        //rManipulatorTrigger.whileTrue(Commands.run(() -> m_IndexerS.setVoltage(Constants.IndexerConstants.indexerVoltage), m_IndexerS).finallyDo(() -> m_IndexerS.setVoltage(0)));
  
        //lManipulatorTrigger.onTrue(Commands.run(() -> m_ShooterS.stop(),m_ShooterS)); //sets the flywheel to 0 speed, shoot button will need pressing again
        
      }
      
    public static ChassisSpeeds fieldOrientedDrive(double x, double y, double rot){
    
      // The origin is always blue. When our alliance is red, X and Y need to be inverted
      
  
      Optional<Alliance> invert = DriverStation.getAlliance();
      Rotation2d gyroDirection = gyro.getRotation2d();
      if (invert.get().equals(Alliance.Red)){
        gyroDirection = gyroDirection.plus(new Rotation2d(Math.PI));
      }
      return ChassisSpeeds.fromFieldRelativeSpeeds(x,y, rot, gyroDirection);
  
    }
  
    public Command getAutonomousCommand() {
      // This method loads the auto when it is called, however, it is recommended
      // to       s load your paths/autos when code starts, then return the
      // pre-loaded auto/path
      //return new PathPlannerAuto("Example Auto");
      return autoChooser.getSelected();
  }

  public double getflWheelVotage(){
    return m_SwerveS.getflTurnVoltage();
  }

  public double getflWheelPos(){
    return m_SwerveS.getflTurnPos();
  }
}
