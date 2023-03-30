package frc.robot.wrappers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.AutoDriveState;
import frc.robot.commands.AutoDriveToTarget;
import frc.robot.commands.AutoFollowTrajectory;
import frc.robot.commands.AutoFollowWithCommands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public final class AutoCommands {
    public final DrivetrainSubsystem m_drivetrainSubsystem;
    public final ArmSubsystem m_armSubsystem;
    public final ManipulatorSubsystem m_manipulatorSubsystem;
    public final AdvancedSetpoints m_advancedSetpoints;

    private final AutoFollowWithCommands autoFollowWithCommands;

    public AutoCommands(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem, AdvancedSetpoints advancedSetpoints) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_armSubsystem = armSubsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;
        m_advancedSetpoints = advancedSetpoints;

        autoFollowWithCommands = new AutoFollowWithCommands(m_drivetrainSubsystem, m_advancedSetpoints);
    }

    public Command autoResetGyro() {
        return new InstantCommand(() -> {m_drivetrainSubsystem.resetGyroscope(180);});
    }

    public Command NoDriveCubeCommand() {
        return m_armSubsystem.m_manipulatorSubsystem.shootCommand().withTimeout(2);
    }

    public Command DriveNoCubeCommand(double x, double y, double a) {
        return Commands.sequence(
            // new InstantCommand(() -> {m_drivetrainSubsystem.driveVector(speed, angle, 0);}),
            // m_drivetrainSubsystem.driveVectorCommand(speed, angle, 0).withTimeout(time)
            new AutoDriveToTarget(m_drivetrainSubsystem, new Pose2d(new Translation2d(x, y), Rotation2d.fromDegrees(a)))
            // new WaitCommand(time),
            // new InstantCommand(m_drivetrainSubsystem::lockDrive)
            // new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(speed, Rotation2d.fromDegrees(angle))).withTimeout(time),
            // new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(0, Rotation2d.fromDegrees(angle)))
        );
    }

    public Command TimeBasedDriveNoCubeCommand(double speed, double angle, double time) {
        return Commands.sequence(
            m_drivetrainSubsystem.driveVectorCommand(-speed, angle, 0, false).withTimeout(time),
            new InstantCommand(m_drivetrainSubsystem::lockDrive)
        );
    }

    public Command LeaveCommunityNoCubeCommand() {
        return Commands.sequence(
            DriveNoCubeCommand(0, 3, 0)
        );
    }

    public Command TimeBasedLeaveCommunityNoCubeCommand() {
        return TimeBasedDriveNoCubeCommand(0.25, 180, 5);
    }

    public Command LeaveCommunityCubeCommand() {
        return Commands.sequence(
            NoDriveCubeCommand(),
            LeaveCommunityNoCubeCommand()
        );
    }

    public Command DockNoCubeCommand() {
        return Commands.sequence(
            DriveNoCubeCommand(0, 2.5, 0),
            new InstantCommand(m_drivetrainSubsystem::lockDrive)
        );
    }

    public Command TimeBasedDockNoCubeCommand() {
        return TimeBasedDriveNoCubeCommand(0.25, 180, 3.5);
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
        // return new AutoFollowTrajectory(m_drivetrainSubsystem, TrajectoryGenerator.generateTrajectory(
        //                 // Start at the origin facing the +X direction
        //                 new Pose2d(0, 0, new Rotation2d(0)),
        //                 // Pass through these two interior waypoints, making an 's' curve path
        //                 List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //                 // End 3 meters straight ahead of where we started, facing forward
        //                 new Pose2d(3, 0, new Rotation2d(0)),
        //                 // Pass config
        //                 Constants.TRAJECTORY_CONFIG), 0.0);
        // return new AutoFollowTrajectory(m_drivetrainSubsystem, TrajectoryGenerator.generateTrajectory(
        //                 // Start at the origin facing the +X direction
        //                 new Pose2d(0, 0, new Rotation2d(0)),
        //                 // Pass through these two interior waypoints, making an straight path
        //                 List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
        //                 // End 3 meters straight ahead of where we started, facing forward
        //                 new Pose2d(4, 0, new Rotation2d(0)),
        //                 // Pass config
        //                 Constants.TRAJECTORY_CONFIG));
        return autoFollowWithCommands.autoFollowWithCommands("test_path");
    }
    public Command runTrajectory(String name){
        return autoFollowWithCommands.autoFollowWithCommands(name);
    }

    public Command HighConeCommand() {
        return Commands.sequence(
            m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.CONE_HIGH)
                .until(m_armSubsystem.isAllAtSetpointBooleanSupplier()),
            m_manipulatorSubsystem.dropCommand()
                .withTimeout(0.5),
            m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.STOW)
                .until(m_armSubsystem.isAllAtSetpointBooleanSupplier())
        );
    }

    public Command MidConeCommand() {
        return Commands.sequence(
            m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.CONE_MID)
                .until(m_armSubsystem.isAllAtSetpointBooleanSupplier()),
            m_manipulatorSubsystem.dropCommand()
                .withTimeout(0.5),
            m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.STOW)
                .until(m_armSubsystem.isAllAtSetpointBooleanSupplier())
        );
    }

    public Command TimeBasedHighConeDockCommand() {
        return Commands.sequence(
            HighConeCommand(),
            TimeBasedDockNoCubeCommand(),
            new InstantCommand(m_drivetrainSubsystem::lockDrive).withTimeout(1),
            autoResetGyro()
        );
    }
    public Command TimeBasedHighConeLeaveCommunityCommand() {
        return Commands.sequence(
            HighConeCommand(),
            TimeBasedLeaveCommunityNoCubeCommand(),
            autoResetGyro()
        );
    }
    public Command TimeBasedMidConeDockCommand() {
        return Commands.sequence(
            MidConeCommand(),
            TimeBasedDockNoCubeCommand(),
            new InstantCommand(m_drivetrainSubsystem::lockDrive).withTimeout(1),
            autoResetGyro()
        );
    }
    public Command TimeBasedMidConeLeaveCommunityCommand() {
        return Commands.sequence(
            MidConeCommand(),
            TimeBasedLeaveCommunityNoCubeCommand(),
            autoResetGyro()
        );
    }
}
