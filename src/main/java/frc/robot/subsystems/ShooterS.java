package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterS extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(Constants.ShooterConstants.shooterMotorID, MotorType.kBrushless);
    private final SparkMax m_motor2 = new SparkMax(Constants.ShooterConstants.shooterMotor2ID, MotorType.kBrushless);
    private final RelativeEncoder shooterEncoder = m_motor.getEncoder();
    private final RelativeEncoder kickupEncoder = m_motor2.getEncoder();
    public ShooterS(){
        SparkMaxConfig config = new SparkMaxConfig();
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_motor2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }
    public void fire(double shooter1Voltage, double shooter2Voltage){
            m_motor.setVoltage(shooter1Voltage);
            m_motor2.setVoltage(shooter2Voltage);
        }
    @Override
    public void periodic(){
        Logger.recordOutput("Shooter/FlywheelRPM", -shooterEncoder.getVelocity());
        Logger.recordOutput("Shooter/KickupRPM", -kickupEncoder.getVelocity());
    }
}

