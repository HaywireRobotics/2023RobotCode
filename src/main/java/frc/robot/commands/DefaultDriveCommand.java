package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;



public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private final CommandXboxController controller;

    //the buffer not only affects the "deadzone", 
    // but also prohibits small angles near the x/y axis
    private static final double buffer = 0.05; 

    public DefaultDriveCommand(DrivetrainSubsystem subsystem, CommandXboxController xboxController) {
        this.m_subsystem = subsystem;
        this.controller = xboxController;

        addRequirements(subsystem);
    }

    private double applyBuffer(double x) {
        if ((x < buffer && x > 0) || (x > -buffer && x < 0)) {
            x = 0;
        }
        // double scaled = (x - buffer) / (1 - buffer);
        // return scaled * scaled;
        return x;
    }

    private double applySmoothing(double x) {
        return Math.pow(x, 3);
    }

    private double applyAll(double x) {
        return applyBuffer(applySmoothing(x));
    }

    @Override
    public void execute() {
        double rightX = controller.getRightX();
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();

        rightX = applyAll(rightX);
        leftX = applyAll(leftX);
        leftY = applyAll(leftY);

        m_subsystem.driveArcade(leftX, leftY, rightX);
      
        /* Odometry */
        m_subsystem.updateOdometry();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.setAllToState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
