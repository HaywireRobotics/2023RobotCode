package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ManipulatorSubsystem;

public class DefaultManipulatorCommand  extends CommandBase{
    private final ManipulatorSubsystem m_subsystem;
    private final XboxController m_controller;
    
    public DefaultManipulatorCommand(ManipulatorSubsystem subsystem, XboxController xboxController){
        m_subsystem = subsystem;
        m_controller = xboxController;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        boolean rightButton = m_controller.getRightBumperPressed();
        boolean leftButton = m_controller.getLeftBumper();
        if(rightButton){
            m_subsystem.intakeCone();
        }else if(leftButton){
            m_subsystem.intakeCube();
        }else{
            m_subsystem.stop();
        }
    }
}
