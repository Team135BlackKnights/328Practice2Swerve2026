package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import frc.robot.Constants.SwerveConstants;


public class SwerveModule {
    private final TalonFX turnMotor; 
    private final TalonFX driveMotor; 
    private final CANcoder turnEncoder; 

    
    
    public SwerveModule(int turnID, int driveID, int encoderID){
        turnMotor = new TalonFX(turnID);
        driveMotor = new TalonFX(driveID);
        turnEncoder = new CANcoder(encoderID);


        final TalonFXConfiguration turnConfigs = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(SwerveConstants.turnNeutralMode)
                .withInverted(SwerveConstants.turnInversion)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
        );

        TalonFXConfigurator turnConfigurator = turnMotor.getConfigurator();
        turnConfigurator.apply(turnConfigs); 


        final TalonFXConfiguration driveConfigs = new TalonFXConfiguration().withMotorOutput(
            new MotorOutputConfigs()
                .withNeutralMode(SwerveConstants.driveNeutralMode)
                .withInverted(SwerveConstants.driveInversion)
        ).withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(Amps.of(120))
                .withStatorCurrentLimitEnable(true)
        );

        TalonFXConfigurator driveConfigurator = driveMotor.getConfigurator();
        driveConfigurator.apply(driveConfigs); 
    }

    public void setDriveVoltage(double voltage){
        driveMotor.setVoltage(voltage);
    }

    public void setTurnVoltage(double voltage){
        turnMotor.setVoltage(voltage);
    }

    public double getTurnPosition(){
        return turnEncoder.getAbsolutePosition().getValueAsDouble();
    }
}
