package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Vector;

public class AutoGravityBalance extends CommandBase{
    
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private double speed = 0.3;
    private double levelError = 0.2;
    private boolean triggeredBalance = false;

    public AutoGravityBalance(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        Vector gravity = getUpwardVector();
        if(gravity.magnitude() < levelError){
            m_drivetrainSubsystem.lockDrive();
            triggeredBalance = true;
            return;
        }
        if(triggeredBalance){
            speed /= 2;
            triggeredBalance = false;
        }
        m_drivetrainSubsystem.driveVector(speed, gravity.direction(), 0, false);
    }

    private Vector getUpwardVector(){
        return m_drivetrainSubsystem.getGyroHorizontalG().scale(-1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
