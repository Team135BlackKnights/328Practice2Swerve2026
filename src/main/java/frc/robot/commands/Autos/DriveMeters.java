package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveS;

public class DriveMeters extends Command{
    boolean isFinished = false;
    final SwerveS swerve;
    final double metersWanted;
    final double degreesWanted;
    static double SPEED_X = 1.5;
    final Pose2d startingPose;
    public DriveMeters(SwerveS m_swerveS, double metersForward, double degreesTurn){
        addRequirements(m_swerveS);
        swerve = m_swerveS;
        metersWanted = metersForward;
        degreesWanted = degreesTurn;
        startingPose = swerve.getPose();
        isFinished = false;
        // SwerveModuleState FLState = new SwerveModuleState(0, Rotation2d.fromDegrees(degreesWanted));
        // SwerveModuleState FRState = new SwerveModuleState(0, Rotation2d.fromDegrees(degreesWanted));
        // SwerveModuleState BLState = new SwerveModuleState(0, Rotation2d.fromDegrees(degreesWanted));
        // SwerveModuleState BRState = new SwerveModuleState(0, Rotation2d.fromDegrees(degreesWanted));
    }
    
    @Override
    public void execute(){
       Pose2d currentPose = swerve.getPose();
       if (startingPose.getTranslation().getDistance(currentPose.getTranslation()) > metersWanted){
        isFinished = true;
       }
       if (metersWanted < 0){
        SPEED_X *= -1;
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
