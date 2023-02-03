package frc.robot.commands;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TestModuleSubsystem;

public class TestModuleCommand extends CommandBase {
    private final TestModuleSubsystem m_subsystem;
    private final SwerveModuleState state;

    public TestModuleCommand(TestModuleSubsystem subsystem, SwerveModuleState state) {
        this.m_subsystem = subsystem;
        this.state = state;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute() {
        m_subsystem.setState(state);
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.stopModule();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
