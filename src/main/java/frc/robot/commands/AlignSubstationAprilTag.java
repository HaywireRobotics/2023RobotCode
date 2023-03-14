package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.wrappers.Camera;

public class AlignSubstationAprilTag extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Camera m_limelight;

    private final double xOffset = 1;
    private final double yOffset = 0;

    public AlignSubstationAprilTag(DrivetrainSubsystem drivetrainSubsystem, Camera limelight) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_limelight = limelight;

        andThen(new PositionAprilTag(drivetrainSubsystem, limelight, xOffset, yOffset, 0));
    }
}
