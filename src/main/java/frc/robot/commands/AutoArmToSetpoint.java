package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ArmAutoPath;
import frc.robot.util.Bezier;

public class AutoArmToSetpoint extends CommandBase{

    private final ArmSubsystem m_armSubsystem;
    private final Bezier targetPath;
    private final double sweepTarget;

    public AutoArmToSetpoint(ArmSubsystem armSubsystem, ArmAutoPath target){
        m_armSubsystem = armSubsystem;
        targetPath = target.path;
        sweepTarget = target.manipulatorAngle;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize(){
        m_armSubsystem.followPath(targetPath);
        m_armSubsystem.setManipulatorHingeTarget(sweepTarget);
    }

    @Override
    public void execute(){
        m_armSubsystem.updateAllPID();
    }

    @Override
    public boolean isFinished(){
        return m_armSubsystem.isAllAtSetpoint();
    }
    
}
