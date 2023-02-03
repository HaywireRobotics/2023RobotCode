package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.TestModuleSubsystem;

public class AutoTestModuleCommand extends SequentialCommandGroup {
    private final TestModuleSubsystem m_subsystem;

    public AutoTestModuleCommand(TestModuleSubsystem subsystem) {
        this.m_subsystem = subsystem;

        andThen(new TestModuleCommand(m_subsystem, 
                new SwerveModuleState(100, Rotation2d.fromDegrees(90))).withTimeout(5));
        andThen(new TestModuleCommand(m_subsystem, 
                new SwerveModuleState(200, Rotation2d.fromDegrees(45))).withTimeout(5));
        andThen(new TestModuleCommand(m_subsystem, 
                new SwerveModuleState(-100, Rotation2d.fromDegrees(120))).withTimeout(5));
        andThen(new TestModuleCommand(m_subsystem, 
                new SwerveModuleState(0, Rotation2d.fromDegrees(0))).withTimeout(5));
        andThen(new InstantCommand(m_subsystem::stopModule));
    }
}
