package frc.robot.subsystems;

import org.w3c.dom.css.CSS2Properties;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeRollerS extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(Constants.IntakeRollerConstants.rollerMotorID, MotorType.kBrushless);
    SlewRateLimiter limiter;
    
    public IntakeRollerS(){
        limiter = new SlewRateLimiter(3);
        SparkMaxConfig config = new SparkMaxConfig();
        config.smartCurrentLimit(200);
        config.idleMode(IdleMode.kCoast);
        m_motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void setVoltage(double rollerVoltage){
        limiter.calculate(rollerVoltage);
        m_motor.setVoltage(rollerVoltage); 
    }
}
