package frc.robot.wrappers;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoArmToSetpoint;
import frc.robot.commands.AutoDriveState;
import frc.robot.commands.AutoDriveToTarget;
import frc.robot.commands.AutoFollowTrajectory;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public final class AutoCommands {
    public final DrivetrainSubsystem m_drivetrainSubsystem;
    public final ArmSubsystem m_armSubsystem;

    public AutoCommands(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_armSubsystem = armSubsystem;
    }

    public Command resetGyroCommand() {
        // return new InstantCommand(() -> {m_drivetrainSubsystem.setGyroOffset(180);});
        return m_drivetrainSubsystem.flipGyroCommand();
    }

    public Command NoDriveCubeCommand() {
        return Commands.sequence(
            m_drivetrainSubsystem.flipGyroCommand(),
            m_armSubsystem.m_manipulatorSubsystem.shootCubeCommand().withTimeout(2)
        );
    }

    public Command DriveNoCubeCommand(double x, double y, double a) {
        return Commands.sequence(
            m_drivetrainSubsystem.flipGyroCommand(),
            // new InstantCommand(() -> {m_drivetrainSubsystem.driveVector(speed, angle, 0);}),
            // m_drivetrainSubsystem.driveVectorCommand(speed, angle, 0).withTimeout(time)
            new AutoDriveToTarget(m_drivetrainSubsystem, new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(a)))
            // new WaitCommand(time),
            // new InstantCommand(m_drivetrainSubsystem::lockDrive)
            // new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(speed, Rotation2d.fromDegrees(angle))).withTimeout(time),
            // new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(0, Rotation2d.fromDegrees(angle)))
        );
    }

    public Command LeaveCommunityNoCubeCommand() {
        return Commands.sequence(
            m_drivetrainSubsystem.flipGyroCommand(),
            DriveNoCubeCommand(0, 3, 0)
        );
    }

    public Command LeaveCommunityCubeCommand() {
        return Commands.sequence(
            NoDriveCubeCommand(),
            LeaveCommunityNoCubeCommand()
        );
    }

    public Command DockNoCubeCommand() {
        return Commands.sequence(
            m_drivetrainSubsystem.flipGyroCommand(),
            DriveNoCubeCommand(0, 2.5, 0),
            new InstantCommand(m_drivetrainSubsystem::lockDrive)
        );
    }

    public Command DockCubeCommand() {
        return Commands.sequence(
            NoDriveCubeCommand(),
            DockNoCubeCommand()
        );
    }

    public Command testAuto(){
        return new AutoDriveToTarget(m_drivetrainSubsystem, new Pose2d(new Translation2d(0, -10), Rotation2d.fromDegrees(0)));
    }

    public Command testTrajectory(){
        return new AutoFollowTrajectory(m_drivetrainSubsystem, TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        new Pose2d(0, 0, new Rotation2d(0)),
                        // Pass through these two interior waypoints, making an 's' curve path
                        List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
                        // End 3 meters straight ahead of where we started, facing forward
                        new Pose2d(3, 0, new Rotation2d(0)),
                        // Pass config
                        Constants.TRAJECTORY_CONFIG), 0.0);
    }
}
