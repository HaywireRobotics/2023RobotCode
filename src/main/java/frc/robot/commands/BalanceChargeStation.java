package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Statics;
import frc.robot.util.Vector;

public class BalanceChargeStation extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;

    private final double acceptedError = 5;

    public BalanceChargeStation(DrivetrainSubsystem drivetrainSubsystem) {
        m_drivetrainSubsystem = drivetrainSubsystem;
    }

    @Override
    public void execute() {
        double roll = m_drivetrainSubsystem.getGyroRoll();
        double correctionSpeed = scaledError(roll);

        if (roll > 0 && roll > acceptedError/2) {
            m_drivetrainSubsystem.setAllToState(new SwerveModuleState(correctionSpeed, Rotation2d.fromDegrees(0)));
        } else if (roll < 0 && roll < -acceptedError/2) {
            m_drivetrainSubsystem.setAllToState(new SwerveModuleState(-correctionSpeed, Rotation2d.fromDegrees(0)));
        }
    }

    private double scaledError(double roll) {
        return Statics.map(Math.abs(roll), acceptedError/2, 45, 0, 0.5);
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.lockDrive();
    }

    @Override
    public boolean isFinished() {
        double roll = m_drivetrainSubsystem.getGyroRoll();
        return (roll < acceptedError) && (roll > -acceptedError);
    }
}
