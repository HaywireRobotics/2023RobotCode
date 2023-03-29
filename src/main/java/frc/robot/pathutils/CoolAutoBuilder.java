package frc.robot.pathutils;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.commands.AutoFollowTrajectory;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CoolAutoBuilder extends BaseAutoBuilder {

    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Map<String, Command> eventMap;
    
    public CoolAutoBuilder(DrivetrainSubsystem drivetrainSubsystem, Map<String, Command> eventMap){
        super(() -> {return new Pose2d();}, (pose) -> {}, eventMap, DrivetrainType.HOLONOMIC, true);

        m_drivetrainSubsystem = drivetrainSubsystem;
        this.eventMap = eventMap;
    }

    @Override
    public CommandBase followPath(PathPlannerTrajectory trajectory) {
        return new AutoFollowTrajectory(m_drivetrainSubsystem, trajectory);
    }
}
