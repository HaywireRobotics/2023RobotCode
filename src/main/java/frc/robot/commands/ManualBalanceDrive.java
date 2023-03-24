package frc.robot.commands;

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
    private static final double JOYSTICK_DEADBAND = 0.1;
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
        double rightX = controller.getRightX();
        double leftX = controller.getLeftX();
        double leftY = controller.getLeftY();

        rightX = applyAll(rightX)*SPEED_SCALE;
        leftX = Statics.applyDeadband(leftX, JOYSTICK_DEADBAND);
        leftY = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);
        Vector leftVector = Statics.applySmoothing2D(new Vector(leftX, leftY), JOYSTICK_S, JOYSTICK_T).scale(SPEED_SCALE);
        leftX = leftVector.x;
        leftY = leftVector.y;
        // System.out.println(leftVector.toString());

        
        // rightX = Statics.applyDeadband(rightX, JOYSTICK_DEADBAND);
        // leftX = Statics.applyDeadband(leftX, JOYSTICK_DEADBAND);
        // leftY = Statics.applyDeadband(leftY, JOYSTICK_DEADBAND);
        // Vector leftVector = new Vector(leftX, leftY);
        // leftX = leftVector.x;
        // leftY = leftVector.y;

        if(leftX == 0 && leftY == 0 && rightX == 0){
            m_subsystem.lockDrive();
         }else {
            m_subsystem.driveArcade(leftX, leftY, rightX);
        };
      
        /* Odometry */
        m_subsystem.updateOdometry();
    }

    @Override
    public void end(boolean interrupted) {
        m_subsystem.lockDrive();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
