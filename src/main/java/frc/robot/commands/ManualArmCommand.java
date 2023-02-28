package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;
    private final CommandJoystick m_controller1;
    private final CommandJoystick m_controller2;

    public ManualArmCommand(ArmSubsystem subsystem, CommandJoystick joystick1, CommandJoystick joystick1){
        m_armSubsystem = subsystem;
        m_controller1 = joystick1;
        m_controller2 = joystick2;

        addRequirements(subsystem);

        configureButtonBindings();
    }

    private void configureButtonBindings(){
        // m_controller.button(0).whileTrue(m_armSubsystem.m_pulleySubsystem.extendCommand());
        // m_controller.button(1).whileTrue(m_armSubsystem.m_pulleySubsystem.retractCommand());

        m_controller1.button(3).whileTrue(m_armSubsystem.m_manipulatorSubsystem.rawUpCommand());
        m_controller1.button(2).whileTrue(m_armSubsystem.m_manipulatorSubsystem.rawDownCommand());

        // m_controller1.button(3).onTrue(m_armSubsystem.m_manipulatorSubsystem.setHingeUpCommand());
        // m_controller1.button(2).onTrue(m_armSubsystem.m_manipulatorSubsystem.setHingeDownCommand());

        m_controller1.button(4).onTrue(m_armSubsystem.m_manipulatorSubsystem.intakeConeCommand());
        m_controller1.button(5).onTrue(m_armSubsystem.m_manipulatorSubsystem.intakeCubeCommand());
        // m_controller.button(1).onTrue(m_armSubsystem.m_manipulatorSubsystem.smartDropCommand());
    }
    @Override
    public void execute() {

        double yAxis1 = m_controller1.getY();
        if(yAxis1 < 0.05 && yAxis1 > -0.05) yAxis1 = 0;

        double yAxis2 = m_controller2.getY();
        if(yAxis2 < 0.05 && yAxis2 > -0.05) yAxis2 = 0;

        boolean extensionButton = m_controller1.button(1).getAsBoolean();
        boolean elevatorButton = m_controller2.button(1).getAsBoolean();

        if(extensionButton){
            m_armSubsystem.setPulleyPower(-yAxis1);
        }else{
            m_armSubsystem.setPulleyPower(0);
        }
        if(elevatorButton){
            m_armSubsystem.setElevatorPower(-yAxis2);
        } else {
            m_armSubsystem.setElevatorPower(0);
        }

    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
