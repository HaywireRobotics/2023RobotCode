package frc.robot.wrappers;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDriveState;
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
        return new InstantCommand(() -> {m_drivetrainSubsystem.setGyroOffset(180);});
    }

    public Command NoDriveCubeCommand() {
        return Commands.sequence(
            resetGyroCommand(),
            m_armSubsystem.m_manipulatorSubsystem.shootCubeCommand().withTimeout(2)
        );
    }

    public Command DriveNoCubeCommand(double speed, double angle, double time) {
        return Commands.sequence(
            resetGyroCommand(),
            // new InstantCommand(() -> {m_drivetrainSubsystem.driveVector(speed, angle, 0);}),
            m_drivetrainSubsystem.driveVectorCommand(speed, angle, 0).withTimeout(time)
            // new WaitCommand(time),
            // new InstantCommand(m_drivetrainSubsystem::lockDrive)
            // new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(speed, Rotation2d.fromDegrees(angle))).withTimeout(time),
            // new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(0, Rotation2d.fromDegrees(angle)))
        );
    }

    public Command LeaveCommunityNoCubeCommand() {
        return Commands.sequence(
            resetGyroCommand(),
            DriveNoCubeCommand(0.25, 180, 5)
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
            resetGyroCommand(),
            DriveNoCubeCommand(0.25, 180, 3.5),
            new InstantCommand(m_drivetrainSubsystem::lockDrive)
        );
    }

    public Command DockCubeCommand() {
        return Commands.sequence(
            NoDriveCubeCommand(),
            DockNoCubeCommand()
        );
    }
}
