package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LoggableTunedNumber;

public class MoveIntakeS extends SubsystemBase {
    private final SparkMax m_motor = new SparkMax(Constants.IntakeConstants.intakeVertMotorID, MotorType.kBrushless);
    RelativeEncoder encoder = m_motor.getEncoder();
    //private final Encoder m_Encoder = new Encoder(Constants.IntakeConstants.intakeVertEncoderID);
    boolean zeroing = false;
    private double zeroSpikeStart = Double.NaN;
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
        double intakeVoltage = intakeController.calculate(encoder.getPosition(), desiredPosition);
        intakeVoltage = clamp(intakeVoltage, -5, 4);
        m_motor.setVoltage(intakeVoltage); 
    }

    public void setVoltage(double voltage){
        //TODO clamp setvoltage
        m_motor.setVoltage(voltage);
    }

    public void zero(){
        zeroing = true;
        zeroSpikeStart = Double.NaN;
    }
    @Override
    public void periodic(){
        LoggableTunedNumber.ifChanged(hashCode(), () -> {
            intakeController = new PIDController(kP.get(), kI.get(), kD.get());
        }, kP, kI, kD);
        if (zeroing){
            double now = Timer.getFPGATimestamp();
            setVoltage(4);
            double observedAmps = Math.abs(m_motor.getOutputCurrent());
            if(observedAmps > 20){
                if (Double.isNaN(zeroSpikeStart)){
                    zeroSpikeStart = now;
                    System.out.println("Spike Detected");
                }
            } else {
                zeroSpikeStart = Double.NaN;
            }
            if (!Double.isNaN(zeroSpikeStart) && (now - zeroSpikeStart) >= 0.1){
                encoder.setPosition(0);
                zeroSpikeStart = Double.NaN;
                zeroing = false;
            }
        }
    }


    // public void vertDownMovement() {
    //     m_motor.setVoltage(0.8);
    // }

    // public void vertUpMovement(){
    //     m_motor.setVoltage(-0.8);
    // }
}