package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSybsystem;

public class DefaultElevatorCommand extends CommandBase {
    private final ElevatorSybsystem m_subsystem;
    private final XboxController m_controller;
    
    public DefaultElevatorCommand(ElevatorSybsystem subsystem, XboxController xboxController){
        m_subsystem = subsystem;
        m_controller = xboxController;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        boolean XButton = m_controller.getXButton();
        boolean YButton = m_controller.getYButton();
        boolean AButton = m_controller.getAButton();
        if(YButton){
            m_subsystem.setExtensionTarget(50);
        }else if(XButton){
            m_subsystem.setExtensionTarget(24);
        }else if(AButton){
            m_subsystem.setExtensionTarget(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
