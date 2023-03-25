package frc.robot.commands;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.EventMarker;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent;
import com.pathplanner.lib.PathPlannerTrajectory.StopEvent.ExecutionBehavior;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.Constants;
import frc.robot.pathutils.CoolAutoBuilder;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.wrappers.AdvancedSetpoints;

public class AutoFollowWithCommands{
    
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final HashMap<String, Command> eventMap = new HashMap<>();
    private final AdvancedSetpoints m_advancedSetpoints;

    public AutoFollowWithCommands(DrivetrainSubsystem drivetrainSubsystem, AdvancedSetpoints advancedSetpoints){
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_advancedSetpoints = advancedSetpoints;

        eventMap.put("intakeCone", new PrintCommand("IntakeCone"));
        eventMap.put("intakeCube", new PrintCommand("IntakeCube"));
        // eventMap.put("intakeCube", m_advancedSetpoints.IntakeCubeCommand());
        eventMap.put("setConeHigh", new PrintCommand("SetConeHigh"));
        // eventMap.put("setConeHigh", m_advancedSetpoints.ArmToSetpoint(Constants.SetpointPositions.CONE_HIGH));

        eventMap.put("stow", new PrintCommand("Stow"));
        // eventMap.put("stow", m_advancedSetpoints.ArmToSetpoint(Constants.SetpointPositions.STOW));
        eventMap.put("drop", new PrintCommand("Drop"));
        // eventMap.put("drop", m_advancedSetpoints.DropGamePiece());

    }

    // public Command autoFollowWithCommands(String name){
    //     ArrayList<PathPlannerTrajectory> path = PathPlanner.loadPathGroup(name, Constants.TRAJECTORY_CONFIG);
    //     System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
    //     System.out.println(eventMap.toString());
    //     for (EventMarker event : path.getMarkers()) {
    //         System.out.println(event.names.toString());
    //     }
    //     return new FollowPathWithEvents(
    //         new AutoFollowTrajectory(m_drivetrainSubsystem, path),
    //         path.getMarkers(),
    //         eventMap
    //     );
    // }

    
    public Command autoFollowWithCommands(String name) {
        List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup(name, Constants.TRAJECTORY_CONFIG);
        return autoFollowFullPath(pathGroup);
    }

    public Command autoFollowFullPath(List<PathPlannerTrajectory> pathGroup){
        CoolAutoBuilder autoBuilder = new CoolAutoBuilder(m_drivetrainSubsystem, eventMap);
        return autoBuilder.fullAuto(pathGroup);
    }
}
