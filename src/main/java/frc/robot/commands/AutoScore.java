package frc.robot.commands;

import java.util.concurrent.Callable;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.Alliances;
import frc.robot.Constants.ScorePositions;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.ArmAutoPath;
import frc.robot.util.Statics;

public class AutoScore{
    

    public static Command autoScoreCommand(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem,
                Alliances alliance, ScorePositions scorePosition, int column, Callable<Pose2d> offset){ 
        
        Pose2d driveTarget = Constants.DriveSetpoints.getGridTargetPose(alliance, column);
        ArmAutoPath armPath = Constants.ArmSetpointPaths.getPathForScorePosition(scorePosition);
        // return Commands.sequence(null)
        return Commands.parallel(
            new AutoArmToSetpoint(armSubsystem, armPath),
            new AutoDriveToTarget(drivetrainSubsystem, Statics.sumPoses(driveTarget, readCallableOffset(offset)))).andThen(
            new WaitCommand(1.0).andThen(
            armSubsystem.m_manipulatorSubsystem.smartDropCommand()
            )
            );
    }
    private static Pose2d readCallableOffset(Callable<Pose2d> offset){
        try{
            return offset.call();
        }catch(Exception e){
            return new Pose2d();
        }
    }
    public static Command autoScoreCommand(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem,
                Alliances alliance, ScorePositions scorePosition, int column){
        return autoScoreCommand(drivetrainSubsystem, armSubsystem, alliance, scorePosition, column, new Pose2d());
    }
    public static Command autoScoreCommand(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem,
                Alliances alliance, ScorePositions scorePosition, int column, Pose2d offset){
        
        return autoScoreCommand(drivetrainSubsystem, armSubsystem, alliance, scorePosition, column, () -> offset);

    }
}
