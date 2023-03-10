package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DrivetrainSubsystem;

public class AutoTestDrivetrain extends SequentialCommandGroup {
    private final DrivetrainSubsystem m_subsystem;

    public AutoTestDrivetrain(DrivetrainSubsystem subsystem) {
        this.m_subsystem = subsystem;

        andThen(new AutoDriveState(m_subsystem, 
                new SwerveModuleState(0, Rotation2d.fromDegrees(0))));
        andThen(new AutoDriveState(m_subsystem, 
                new SwerveModuleState(0, Rotation2d.fromDegrees(45))).withTimeout(5));
        andThen(new AutoDriveState(m_subsystem, 
                new SwerveModuleState(0, Rotation2d.fromDegrees(120))).withTimeout(5));
        andThen(new AutoDriveState(m_subsystem, 
                new SwerveModuleState(0, Rotation2d.fromDegrees(0))).withTimeout(5));
    }
}
