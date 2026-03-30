package frc.robot.commands.Autos;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj2.command.Commands;
//import frc.robot.Constants;
import frc.robot.subsystems.IndexerS;
import frc.robot.subsystems.ShooterS;
import frc.robot.subsystems.SwerveS;

public class DriveMeters extends Command{
    boolean isFinished = false;
    final SwerveS swerve;
    final IndexerS indexer;
    final double metersWanted;
    final double degreesWanted;
    static double SPEED_X = 1.5;
    final Pose2d startingPose;
    final ShooterS shooter;
    SwerveModuleState[] states;
    Rotation2d twist;    
    
    public DriveMeters(SwerveS m_swerveS, ShooterS m_shooterS, IndexerS m_indexerS, double metersForward, double degreesTurn){
        addRequirements(m_swerveS);
        swerve = m_swerveS;
        shooter = m_shooterS;
        indexer = m_indexerS;
        metersWanted = metersForward;
        degreesWanted = degreesTurn;
        startingPose = swerve.getPose();
        isFinished = false;
        twist = new Rotation2d(Math.toRadians(degreesTurn));
        states[0] = new SwerveModuleState(0, twist);
        states[1] = new SwerveModuleState(0, twist.plus(new Rotation2d(Math.toRadians(90))));
        states[2] = new SwerveModuleState(0, twist.plus(new Rotation2d(Math.toRadians(180))));
        states[3] = new SwerveModuleState(0, twist.plus(new Rotation2d(Math.toRadians(270))));
    }
    
    
    @Override
    public void execute(){
       Pose2d currentPose = swerve.getPose();
       if (startingPose.getTranslation().getDistance(currentPose.getTranslation()) > metersWanted){
        isFinished = true;
       }
       if (metersWanted < 0){
        SPEED_X *= -1;
        swerve.setModuleStates(states[0], states[1], states[2], states[3]);
        //Commands.run(() -> shooter.fire(Constants.ShooterConstants.shooter1Voltage, Constants.ShooterConstants.shooter2voltage), shooter).finallyDo(() -> shooter.fire(0,0)).raceWith(Commands.run(() -> indexer.setVoltage(Constants.IndexerConstants.indexerVoltage), indexer).finallyDo(() -> indexer.setVoltage(0))).withTimeout(5 /* seconds */);
       }
       swerve.setSpeed(SPEED_X, 0, 0);
    }
    @Override
    public void end(boolean interrupted){
        swerve.setSpeed(0, 0, 0);
    }
    @Override
    public boolean isFinished(){
        return isFinished;
    }

}
