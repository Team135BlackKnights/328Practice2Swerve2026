package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class MoveIntakeS extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(Constants.IntakeConstants.intakeVertMotorID, MotorType.kBrushless);
    private final DutyCycleEncoder m_Encoder = new DutyCycleEncoder(Constants.IntakeConstants.intakeVertEncoderID);

    private PIDController intakeController = new PIDController(Constants.IntakeConstants.intakePID[0], Constants.IntakeConstants.intakePID[1], Constants.IntakeConstants.intakePID[2]);

    
    public void moveTo(double desiredPosition){
        double intakeVoltage = intakeController.calculate(m_Encoder.get(), desiredPosition);
        intakeVoltage = MathUtil.clamp(intakeVoltage, Constants.IntakeConstants.intakeDownVoltage, Constants.IntakeConstants.intakeUpVoltage);
        m_motor.setVoltage(intakeVoltage); 
    }

    public void setVoltage(double voltage){
        voltage = MathUtil.clamp(voltage, Constants.IntakeConstants.intakeDownVoltage, Constants.IntakeConstants.intakeUpVoltage);
        m_motor.setVoltage(voltage);
    }




    // public void vertDownMovement() {
    //     m_motor.setVoltage(0.8);
    // }

    // public void vertUpMovement(){
    //     m_motor.setVoltage(-0.8);
    // }
}