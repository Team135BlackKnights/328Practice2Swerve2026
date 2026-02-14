package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IndexerS extends SubsystemBase{
    private final TalonFX motor = new TalonFX(Constants.IndexerConstants.indexerMotorID); // CAN ID

    public void setVoltage(double indexerVoltage) {
        motor.setVoltage(indexerVoltage);
}
}