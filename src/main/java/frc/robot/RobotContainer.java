// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.MoveIntakeDownC;
import frc.robot.commands.SwerveC;
import frc.robot.subsystems.IntakeS;
import frc.robot.subsystems.SwerveS;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


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
  
  public static final Trigger xDriverButtonTrigger = m_driverController.x();
  public static final Trigger yDriverButtonTrigger = m_driverController.y();
  public static final Trigger aDriverButtonTrigger = m_driverController.a();
  public static final Trigger bDriverButtonTrigger = m_driverController.b();
  
  public static final Trigger xManipulatorButtonTrigger = m_manipulatorController.x();
  public static final Trigger yManipulatorButtonTrigger = m_manipulatorController.y();
  public static final Trigger aManipulatorButtonTrigger = m_manipulatorController.a();
  public static final Trigger bManipulatorButtonTrigger = m_manipulatorController.b();


  SwerveS m_SwerveS = new SwerveS();
  IntakeS m_IntakeS = new IntakeS();

  SwerveC m_SwerveC = new SwerveC(m_SwerveS);
  MoveIntakeDownC m_MoveIntakeDownC = new MoveIntakeDownC(m_IntakeS);

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


    //xButtonTrigger.whileTrue(m_IntakeC); /TODO - this is how you do that

    aManipulatorButtonTrigger.whileTrue(m_MoveIntakeDownC);


  }
}
