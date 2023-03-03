package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

public class CoolAutoCommandCharge extends CommandBase {
    public final DrivetrainSubsystem m_drivetrainSubsystem;
    public final ArmSubsystem m_armSubsystem;

    public CoolAutoCommandCharge(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_armSubsystem = armSubsystem;

        andThen(new InstantCommand(m_armSubsystem.m_manipulatorSubsystem::shootCube).withTimeout(2));
        andThen(new InstantCommand(m_armSubsystem.m_manipulatorSubsystem::stop));
        // andThen(new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState()))
        andThen(new AutoDriveToTarget(m_drivetrainSubsystem, new Pose2d(new Translation2d(0.0, 5.4), new Rotation2d(180)))); 
    }
    
}
