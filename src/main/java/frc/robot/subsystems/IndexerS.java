package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Amps;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;

public class IndexerS extends SubsystemBase{
    private final TalonFX motor = new TalonFX(Constants.IndexerConstants.indexerMotorID); // CAN ID

    public IndexerS(){
        final TalonFXConfiguration config = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(20))
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(20)
                .withSupplyCurrentLimitEnable(true)
        );
        TalonFXConfigurator configurator = motor.getConfigurator();
        configurator.apply(config);
    }

    public void setVoltage(double indexerVoltage) {
        motor.setVoltage(indexerVoltage);
    }

    public void periodic(){
        Logger.recordOutput("Indexer/Voltage", motor.getMotorVoltage().getValueAsDouble());
    }
}