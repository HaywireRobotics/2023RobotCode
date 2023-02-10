package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.wrappers.Camera;

public class CenterToAprilTag extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private final Camera m_camera;

    private final PIDController centerController;

    private final double CENTER_KP = 4;
    private final double CENTER_KI = 0;
    private final double CENTER_KD = 0;

    public CenterToAprilTag(DrivetrainSubsystem drivetrainSubsystem, Camera camera) {
        m_drivetrainSubsystem = drivetrainSubsystem;
        m_camera = camera;

        centerController = new PIDController(CENTER_KP, CENTER_KI, CENTER_KD);
    }

    @Override
    public void execute() {
        if (!m_camera.hasTargets()) {
            System.out.println("No April Tag Detected");
            return;
        }

        Transform2d target2D = m_camera.getTarget2D();

        double error = target2D.getY();
        double output = centerController.calculate(error, 0);

        m_drivetrainSubsystem.driveVector(output, 90, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
