package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import org.opencv.core.Mat;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Hopper extends SubsystemBase{
    private final SparkMax m_motor = new SparkMax(Constants.HoodConstants.hoodMotorID, MotorType.kBrushless);
    private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(Constants.HoodConstants.hoodEncoderID);
    private final PIDController m_Controller = new PIDController(Constants.HopperConstants.hopperPID[0], Constants.HopperConstants.hopperPID[1], Constants.HopperConstants.hopperPID[2]); 

    public void setVoltage(double v) {
        m_motor.setVoltage(MathUtil.clamp(v, 0, Constants.HopperConstants.hopperMaxVoltage));
    }

    public void extendHopper() {
        double voltage = m_Controller.calculate(m_Encoder.get(), Constants.HopperConstants.outSetpoint);
        voltage = MathUtil.clamp(voltage, 0, Constants.HopperConstants.hopperMaxVoltage);
        m_motor.setVoltage(voltage);
    }

    public void retractHopper() {
        double voltage = m_Controller.calculate(m_Encoder.get(), Constants.HopperConstants.inSetpoint);
        voltage = MathUtil.clamp(voltage, 0, Constants.HopperConstants.hopperMaxVoltage);
        m_motor.setVoltage(voltage);
    }
}
