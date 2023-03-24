package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoFollowWithCommands{
    
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final HashMap<String, Command> eventMap = new HashMap<>();

    public AutoFollowWithCommands(DrivetrainSubsystem drivetrainSubsystem){
        m_drivetrainSubsystem = drivetrainSubsystem;

        eventMap.put("IntakeCone", new PrintCommand("IntakeCone"));
        eventMap.put("SetConeHigh", new PrintCommand("SetConeHigh"));
        eventMap.put("Stow", new PrintCommand("Stow"));
        eventMap.put("Drop", new PrintCommand("Drop"));

    }

    public Command autoFollowWithCommands(String name){
        PathPlannerTrajectory examplePath = PathPlanner.loadPath("1.5p_2g_c", Constants.TRAJECTORY_CONFIG);

        return new FollowPathWithEvents(
            new AutoFollowTrajectory(m_drivetrainSubsystem, examplePath),
            examplePath.getMarkers(),
            eventMap
        );
    }

}
