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
    private final SparkMax m_motor = new SparkMax(Constants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(Constants.ShooterConstants.shooterMotor2ID, MotorType.kBrushless);
    private final PIDController shooterController = new PIDController(0, 0, 0);
    private final PIDController kickupController = new PIDController(0, 0, 0);
    private final RelativeEncoder kickupEncoder = m_motor.getEncoder();
    private final RelativeEncoder shooterEncoder = m_motor2.getEncoder();
    public ShooterS(){
        SparkMaxConfig config = new SparkMaxConfig();
        config.voltageCompensation(12);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void fire(double kickupVoltage, double flywheelVoltage){
            kickupVoltage = kickupController.calculate(kickupVoltage);
            flywheelVoltage = shooterController.calculate(flywheelVoltage);
            m_motor.setVoltage(kickupVoltage);
            m_motor2.setVoltage(flywheelVoltage);
        }
    @Override
    public void periodic(){
        Logger.recordOutput("Shooter/KickupRPM", Math.abs(kickupEncoder.getVelocity()/5));
        Logger.recordOutput("Shooter/ShooterRPM", Math.abs(shooterEncoder.getVelocity()));
        Logger.recordOutput("Shooter/ShooterAmps", m_motor2.getOutputCurrent());
        Logger.recordOutput("Shooter/KickupAmps", m_motor.getOutputCurrent());
    }
}

