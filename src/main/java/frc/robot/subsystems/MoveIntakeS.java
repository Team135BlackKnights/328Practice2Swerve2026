package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggableTunedNumber;

public class MoveIntakeS extends SubsystemBase {
    private final SparkMax motor = new SparkMax(Constants.IntakeConstants.intakeVertMotorID, MotorType.kBrushless);
    private final DutyCycleEncoder encoder = new DutyCycleEncoder(0);
    private final RelativeEncoder relencoder = motor.getEncoder();
    double offset = 0.935;
    //private final Encoder m_Encoder = new Encoder(Constants.IntakeConstants.intakeVertEncoderID);
    boolean zeroing = false;
    //private double zeroSpikeStart = Double.NaN;
    private PIDController intakeController = new PIDController(Constants.IntakeConstants.intakePID[0], Constants.IntakeConstants.intakePID[1], Constants.IntakeConstants.intakePID[2]);
    private final LoggableTunedNumber kP = new LoggableTunedNumber("Intake/kP",Constants.IntakeConstants.intakePID[0],true);
    private final LoggableTunedNumber kI = new LoggableTunedNumber("Intake/kI",Constants.IntakeConstants.intakePID[1],true);
    private final LoggableTunedNumber kD = new LoggableTunedNumber("Intake/kD",Constants.IntakeConstants.intakePID[2],true);
    private double clamp(double a, double min, double max){
        return  Math.max(Math.min(a, max), min);
    }

    public void moveTo(double desiredPosition){
        if (zeroing){
            return;
        }
        double intakeVoltage = intakeController.calculate(getEncoderPositionWithOffset(), desiredPosition);
        intakeVoltage = clamp(intakeVoltage, -5, 4);
        setVoltage(intakeVoltage); 
    }

    public void setVoltage(double voltage){
        voltage = MathUtil.clamp(voltage, -5, 4);
        motor.setVoltage(voltage);
    }

    public void zero(){
        zeroing = true;
        //zeroSpikeStart = Double.NaN;
    }
    @Override
    public void periodic(){
        // setVoltage(-3);        
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            intakeController = new PIDController(kP.get(), kI.get(), kD.get());
        }, kP, kI, kD);
        // if (zeroing){
        //     double now = Timer.getFPGATimestamp();
        //     setVoltage(4);
        //     double observedAmps = Math.abs(m_motor.getOutputCurrent());
        //     if(observedAmps > 20){
        //         if (Double.isNaN(zeroSpikeStart)){
        //             zeroSpikeStart = now;
        //             System.out.println("Spike Detected");
        //         }
        //     } else {
        //         zeroSpikeStart = Double.NaN;
        //     }
        //     if (!Double.isNaN(zeroSpikeStart) && (now - zeroSpikeStart) >= 0.1){
        //         offset = encoder.get();
        //         zeroSpikeStart = Double.NaN;
        //         zeroing = false;
        //     }
        // }
        Logger.recordOutput("Intake/Encoder pos", encoder.get());
        Logger.recordOutput("Intake/Encoder with offset", getEncoderPositionWithOffset());
        Logger.recordOutput("Intake/Encoder connected", encoder.isConnected());
        Logger.recordOutput("Intake/RelEncoder pos", relencoder.getPosition());
    }

    public double getEncoderPositionWithOffset(){
        return encoder.get() - offset;
    }

    // public void vertDownMovement() {
    //     m_motor.setVoltage(0.8);
    // }

    // public void vertUpMovement(){
    //     m_motor.setVoltage(-0.8);
    // }
}