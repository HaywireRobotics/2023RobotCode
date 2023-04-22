package frc.robot.commands;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.pathutils.CoolAutoBuilder;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.wrappers.AdvancedSetpoints;

public class AutoFollowWithCommands{
    
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final HashMap<String, Command> eventMap = new HashMap<>();
    private final AdvancedSetpoints m_advancedSetpoints;
    private boolean requireDriveLock = false;

    private Command drivingCommand;

    public AutoFollowWithCommands(DrivetrainSubsystem drivetrainSubsystem, AdvancedSetpoints advancedSetpoints){
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_advancedSetpoints = advancedSetpoints;

        // eventMap.put("intakeCone", new PrintCommand("IntakeCone"));
        eventMap.put("intakeCone", m_advancedSetpoints.IntakeConeCommand().asProxy());
        // eventMap.put("intakeCube", new PrintCommand("IntakeCube"));
        eventMap.put("intakeCube", m_advancedSetpoints.IntakeCubeCommand().asProxy());
        // eventMap.put("setConeHigh", new PrintCommand("SetConeHigh"));
        eventMap.put("setConeHigh", m_advancedSetpoints.ArmToSetpoint(Constants.SetpointPositions.CONE_HIGH).asProxy());

        // eventMap.put("stow", new PrintCommand("Stow"));
        eventMap.put("stow", m_advancedSetpoints.ArmToSetpoint(Constants.SetpointPositions.STOW).asProxy());
        // eventMap.put("stow", new ProxyCommand(new ScheduleCommand( m_advancedSetpoints.ArmToSetpoint(Constants.SetpointPositions.STOW)) ));//.andThen( new WaitCommand(1.5) ));
        // eventMap.put("drop", new PrintCommand("Drop"));
        eventMap.put("drop", m_advancedSetpoints.DropGamePiece().withTimeout(0.5).asProxy());
        // eventMap.put("substation", new PrintCommand("Substation"));
        // eventMap.put("substation", enableDriveLock());
        eventMap.put("substation", lockDriveCommand().asProxy());
        eventMap.put("noEvent", new InstantCommand());

        m_advancedSetpoints.setArmPIDDefault();

    }
    
    public Command autoFollowWithCommands(String name) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(name, Constants.TRAJECTORY_CONFIG);
        for(int i = 0; i < pathGroup.size(); i++){
            pathGroup.set(i, PathPlannerTrajectory.transformTrajectoryForAlliance(pathGroup.get(i), DriverStation.getAlliance()));
        };
        return autoFollowFullPath(pathGroup);
    }

    public Command autoFollowFullPath(List<PathPlannerTrajectory> pathGroup){
        CoolAutoBuilder autoBuilder = new CoolAutoBuilder(m_drivetrainSubsystem, eventMap);
        drivingCommand =  autoBuilder.fullAuto(pathGroup).andThen(new PrintCommand("qwriotqhtowqherothjwjoirghjioejriowhjoifthweoirhj"));
        return drivingCommand;
    }

    private Command enableDriveLock(){
        return new InstantCommand(() -> requireDriveLock=true).andThen(new InstantCommand(() -> drivingCommand.cancel())).andThen(lockDriveCommand());
    }
    private Command lockDriveIfRequired(){
        return new InstantCommand(() -> {
            if(requireDriveLock){
                m_drivetrainSubsystem.lockDrive();
            }
        });
    }
    private Command lockDriveCommand() {
        return new RunCommand(m_drivetrainSubsystem::lockDrive, m_drivetrainSubsystem).withTimeout(3);
    }
}
