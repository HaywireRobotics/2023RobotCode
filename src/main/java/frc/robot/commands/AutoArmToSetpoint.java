package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.ArmAutoPath;
import frc.robot.util.Bezier;

public class AutoArmToSetpoint extends CommandBase{

    private final ArmSubsystem m_armSubsystem;
    private final double flipTime;
    private final Bezier targetPath;
    private final double sweepTarget;
    private final double startAngleTarget = 15;
    
    public AutoArmToSetpoint(ArmSubsystem armSubsystem, ArmAutoPath target){
        m_armSubsystem = armSubsystem;
        targetPath = target.path;
        sweepTarget = target.manipulatorAngle;
        flipTime = target.flipTime;

        addRequirements(m_armSubsystem);
    }

    @Override
    public void initialize(){
        m_armSubsystem.followPath(targetPath);
        m_armSubsystem.setManipulatorHingeTarget(startAngleTarget);
        m_armSubsystem.isPathFollowing = true;
    }

    @Override
    public void execute(){
        m_armSubsystem.updateAllPID();

        if ( m_armSubsystem.followT > flipTime) {
            m_armSubsystem.setManipulatorHingeTarget(sweepTarget);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_armSubsystem.endPath();
        m_armSubsystem.stabilizeArm();
        m_armSubsystem.isPathFollowing = false;
    }

    @Override
    public boolean isFinished(){
        return m_armSubsystem.isAllAtSetpoint();
    }
    
}
