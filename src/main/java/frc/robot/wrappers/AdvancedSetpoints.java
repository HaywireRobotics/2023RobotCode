package frc.robot.wrappers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.SetpointPositions;
import frc.robot.commands.AutoArmToSetpoint;
import frc.robot.commands.AutoDriveToTarget;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class AdvancedSetpoints {
    
    public final DrivetrainSubsystem m_drivetrainSubsystem;
    public final ArmSubsystem m_armSubsystem;
    public final ManipulatorSubsystem m_manipulatorSubsystem;

    public AdvancedSetpoints(DrivetrainSubsystem drivetrainSubsystem, ArmSubsystem armSubsystem, ManipulatorSubsystem manipulatorSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_armSubsystem = armSubsystem;
        m_manipulatorSubsystem = manipulatorSubsystem;
    }


    public Command substationCommand(){
        Constants.Alliances alliance = DriverStation.getAlliance() == DriverStation.Alliance.Blue ? Constants.Alliances.BLUE : Constants.Alliances.RED;
        Pose2d driveGoal = getNearestSubstation(alliance);
        return new SequentialCommandGroup(
            // new ParallelCommandGroup(
            //     new AutoDriveToTarget(m_drivetrainSubsystem, driveGoal),
                new AutoArmToSetpoint(m_armSubsystem, Constants.ArmSetpointPaths.SUBSTATION)
                .until(m_armSubsystem.isAllAtSetpointBooleanSupplier()),
            // ),
            m_manipulatorSubsystem.intakeCommand(),
            new InstantCommand(() -> {m_armSubsystem.setManipulator2dPosition(Constants.ArmSetpoints.SUBSTATION.armPosition.x, Constants.ArmSetpoints.SUBSTATION.armPosition.y-5);}),
            m_manipulatorSubsystem.setHingeTargetCommand(150),
            new WaitCommand(0.5),
            m_manipulatorSubsystem.stopCommand(),
            new AutoArmToSetpoint(m_armSubsystem, Constants.ArmSetpointPaths.STOW)
        );
    }

    private Pose2d getNearestDriveTarget(Constants.Alliances alliance){
        Translation2d currentPosition = m_drivetrainSubsystem.getPose().getTranslation();
        Pose2d closest = Constants.DriveSetpoints.BlueSubstation[0];
        double closestDistance = currentPosition.getDistance(closest.getTranslation());

        Pose2d[] targets = new Pose2d[Constants.DriveSetpoints.BlueSubstation.length+Constants.DriveSetpoints.BlueGrid.length];
        
        System.arraycopy(Constants.DriveSetpoints.getSubstationTargets(alliance), 0, targets, 0, Constants.DriveSetpoints.BlueSubstation.length);
        System.arraycopy(Constants.DriveSetpoints.getGridTargets(alliance), 0, targets, Constants.DriveSetpoints.BlueSubstation.length, Constants.DriveSetpoints.BlueGrid.length);

        for (Pose2d target : targets){
            double distance = currentPosition.getDistance(closest.getTranslation());
            if (distance < closestDistance){
                closest = target;
                closestDistance = distance;
            }
        }
        return closest;
    }

    private Pose2d getNearestSubstation(Constants.Alliances alliance){
        Translation2d currentPosition = m_drivetrainSubsystem.getPose().getTranslation();
        Pose2d closest = Constants.DriveSetpoints.getSubstationTargets(alliance)[0];
        double closestDistance = currentPosition.getDistance(closest.getTranslation());

        for (Pose2d target : Constants.DriveSetpoints.getSubstationTargets(alliance)){
            double distance = currentPosition.getDistance(closest.getTranslation());
            if (distance < closestDistance){
                closest = target;
                closestDistance = distance;
            }
        }
        return closest;
    }

    private Pose2d getNearestGrid(Constants.Alliances alliance){
        Translation2d currentPosition = m_drivetrainSubsystem.getPose().getTranslation();
        Pose2d closest = Constants.DriveSetpoints.getGridTargets(alliance)[0];
        double closestDistance = currentPosition.getDistance(closest.getTranslation());

        for (Pose2d target : Constants.DriveSetpoints.getGridTargets(alliance)){
            double distance = currentPosition.getDistance(closest.getTranslation());
            if (distance < closestDistance){
                closest = target;
                closestDistance = distance;
            }
        }
        return closest;
    }

    public Command IntakeCubeCommand() {
        return Commands.sequence(
            m_armSubsystem.m_manipulatorSubsystem.startIntakeCommand(),
            m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.CUBE_PICKUP)
                .until(m_armSubsystem.isAllAtSetpointBooleanSupplier()),
            m_armSubsystem.m_manipulatorSubsystem.intakeCommand()
                .withTimeout(0.5),
            m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.STOW)
                .until(m_armSubsystem.isAllAtSetpointBooleanSupplier())
        );
    }

    public Command ArmToSetpoint(SetpointPositions setpoint){
        return m_armSubsystem.adaptiveSetpointCommand(setpoint);
    }
    public Command DropGamePiece(){
        return m_armSubsystem.m_manipulatorSubsystem.dropCommand();
    }
}
