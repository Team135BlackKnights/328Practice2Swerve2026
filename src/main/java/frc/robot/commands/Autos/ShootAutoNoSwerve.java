package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IndexerS;
import frc.robot.subsystems.MoveIntakeS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.SwerveS;
import frc.robot.Constants;

public class ShootAutoNoSwerve extends Command{
    final IndexerS indexer;
    final ShooterS shooter;
    final MoveIntakeS moveintake;
    final SwerveS swerve;

    public ShootAutoNoSwerve(IndexerS m_IndexerS, ShooterS m_ShooterS, MoveIntakeS m_MoveIntakeS, SwerveS m_SwerveS){
        indexer = m_IndexerS;
        shooter = m_ShooterS;
        moveintake = m_MoveIntakeS;
        swerve = m_SwerveS;
    }

    public void execute(){
        swerve.setSpeed(0,0,0);
        moveintake.zero();
        shooter.fireControlledSpeed(Constants.ShooterConstants.constantKickupVoltage);
        indexer.setVoltage(Constants.IndexerConstants.indexerVoltage);
        
    }
    public void end(){
        shooter.idle(-4);
        indexer.setVoltage(0);
    }
}
