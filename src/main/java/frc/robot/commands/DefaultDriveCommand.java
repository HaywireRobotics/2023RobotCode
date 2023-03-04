package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Statics;
import frc.robot.util.Vector;



public class DefaultDriveCommand extends CommandBase {
    private final DrivetrainSubsystem m_subsystem;
    private final CommandXboxController controller;

    //the buffer not only affects the "deadzone", 
    // but also prohibits small angles near the x/y axis
    private static final double JOYSTICK_DEADBAND = 0.15;
    private static final double JOYSTICK_S = 0.2;
    private static final double JOYSTICK_T = 1.4;

    public DefaultDriveCommand(DrivetrainSubsystem subsystem, CommandXboxController xboxController) {
        this.m_subsystem = subsystem;
        this.controller = xboxController;

        addRequirements(subsystem);
    }

    private double applyAll(double x) {
        return Statics.applySmoothing1D(Statics.applyDeadband(x, JOYSTICK_DEADBAND), JOYSTICK_S, JOYSTICK_T);
    }

    @Override
    public void execute() {
        double rightX = controller.getRightX();
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();

        // rightX = applyAll(rightX);
        // leftX = Statics.applyDeadband(leftX, JOYSTICK_DEADBAND);
        // leftY = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);
        // rightX = applyAll(rightX);
        rightX = Statics.applyDeadband(JOYSTICK_DEADBAND, rightX);
        leftX = Statics.applyDeadband(JOYSTICK_DEADBAND, leftX);
        leftY = Statics.applyDeadband(JOYSTICK_DEADBAND, leftY);
        Vector leftVector = new Vector(leftX, leftY);//Statics.applySmoothing2D(new Vector(leftX, leftY), JOYSTICK_S, JOYSTICK_T);
        leftX = leftVector.x;
        leftY = leftVector.y;
        // System.out.println(leftVector.toString());

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
