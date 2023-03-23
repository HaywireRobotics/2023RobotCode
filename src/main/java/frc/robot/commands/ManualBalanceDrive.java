package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.util.Statics;
import frc.robot.util.Vector;

public class ManualBalanceDrive extends CommandBase{
    private final DrivetrainSubsystem m_subsystem;
    private final CommandXboxController controller;

    //the buffer not only affects the "deadzone", 
    // but also prohibits small angles near the x/y axis
    private static final double JOYSTICK_DEADBAND = 0.2;
    private static final double JOYSTICK_S = 0.1;
    private static final double JOYSTICK_T = 2.6;
    private static final double SPEED_SCALE = 0.25;

    public ManualBalanceDrive(DrivetrainSubsystem subsystem, CommandXboxController xboxController) {
        this.m_subsystem = subsystem;
        this.controller = xboxController;

        addRequirements(subsystem);
    }

    private double applyAll(double x) {
        return Statics.applySmoothing1D(Statics.applyDeadband(x, JOYSTICK_DEADBAND), JOYSTICK_S, JOYSTICK_T);
    }

    @Override
    public void execute() {
        double rightX = controller.getRightX()*SPEED_SCALE;
        double leftX = controller.getLeftX()*SPEED_SCALE;
        double leftY = controller.getLeftY()*SPEED_SCALE;

        rightX = applyAll(rightX);
        leftX = Statics.applyDeadband(leftX, JOYSTICK_DEADBAND);
        leftY = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);
        Vector leftVector = Statics.applySmoothing2D(new Vector(leftX, leftY), JOYSTICK_S, JOYSTICK_T);
        leftX = leftVector.x;
        leftY = leftVector.y;
        // System.out.println(leftVector.toString());

        
        // rightX = Statics.applyDeadband(rightX, JOYSTICK_DEADBAND);
        // leftX = Statics.applyDeadband(leftX, JOYSTICK_DEADBAND);
        // leftY = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);
        // Vector leftVector = new Vector(leftX, leftY);
        // leftX = leftVector.x;
        // leftY = leftVector.y;

        m_subsystem.driveArcade(leftX, leftY, rightX);

        if(leftX == 0 && leftY == 0 && rightX == 0){
            lockDrive();
         }else {
            m_subsystem.driveArcade(leftX, leftY, rightX);
        };
      
        /* Odometry */
        m_subsystem.updateOdometry();
    }
    private void lockDrive(){
        m_subsystem.setBackLeft(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        m_subsystem.setBackRight(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
        m_subsystem.setFrontLeft(new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0)));
        m_subsystem.setFrontRight(new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0)));
    }

    @Override
    public void end(boolean interrupted) {
        lockDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
