package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterS extends SubsystemBase{
    private final SparkMax kickMotor = new SparkMax(Constants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
    private final SparkMax shootMotor = new SparkMax(Constants.ShooterConstants.shooterMotor2ID, MotorType.kBrushless);
    private final PIDController shooterController = new PIDController(5, 0, 0);
    private final PIDController kickupController = new PIDController(2, 0, 0);
    private final RelativeEncoder kickupEncoder = kickMotor.getEncoder();
    private final RelativeEncoder shooterEncoder = shootMotor.getEncoder();
    public ShooterS(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        //make higher if motor no work
        config.smartCurrentLimit(65);
        kickMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        shootMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void fire(double kickupVoltage, double flywheelVoltage){
            kickupVoltage = kickupController.calculate(kickupVoltage);
            flywheelVoltage = shooterController.calculate(flywheelVoltage);
            kickMotor.setVoltage(kickupVoltage);
            shootMotor.setVoltage(flywheelVoltage);
        }
    @Override
    public void periodic(){
        //Note: the kickup and shooter RPMs lie to you, put an absolute encoder on the flywheel plz
        Logger.recordOutput("Shooter/KickupRPM", Math.abs(kickupEncoder.getVelocity()));
        Logger.recordOutput("Shooter/ShooterRPM", Math.abs(shooterEncoder.getVelocity()));
        Logger.recordOutput("Shooter/ShooterAmps", shootMotor.getOutputCurrent());
        Logger.recordOutput("Shooter/KickupAmps", kickMotor.getOutputCurrent());
    }
}

