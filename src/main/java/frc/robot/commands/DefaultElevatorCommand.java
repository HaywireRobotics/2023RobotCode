package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ElevatorSybsystem;

public class DefaultElevatorCommand extends CommandBase {
    private final ElevatorSybsystem m_subsystem;
    private final Joystick m_joystick;
    
    public DefaultElevatorCommand(ElevatorSybsystem subsystem, Joystick joystick){
        m_subsystem = subsystem;
        m_joystick = joystick;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        boolean button6 = m_joystick.getRawButton(6);
        boolean button7 = m_joystick.getRawButton(7);
        boolean button8 = m_joystick.getRawButton(8);
        if(button7){
            m_subsystem.setTarget(50);
        }else if(button6){
            m_subsystem.setTarget(24);
        }else if(button8){
            m_subsystem.setTarget(0);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
