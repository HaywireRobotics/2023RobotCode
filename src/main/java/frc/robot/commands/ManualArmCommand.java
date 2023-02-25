package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;
    private final CommandXboxController m_controller;

    public ManualArmCommand(ArmSubsystem subsystem, CommandXboxController xboxController){
        m_armSubsystem = subsystem;
        m_controller = xboxController;

        addRequirements(subsystem);
    }

    @Override
    public void execute() {
        boolean upButton = m_controller.povUp().getAsBoolean();
        boolean downButton = m_controller.povDown().getAsBoolean();
        boolean leftButton = m_controller.povLeft().getAsBoolean();
        boolean rightButton = m_controller.povRight().getAsBoolean();

        boolean leftBumper = m_controller.leftBumper().getAsBoolean();
        boolean leftTrigger = m_controller.leftTrigger().getAsBoolean();
        boolean rightTrigger = m_controller.leftTrigger().getAsBoolean();

        if(upButton){
            m_armSubsystem.setElevatorPower(0.2);
        }else if(downButton){
            m_armSubsystem.setElevatorPower(-0.2);
        }else{
            m_armSubsystem.setElevatorPower(0);
        }

        if(leftButton){
            m_armSubsystem.setPulleyPower(0.2);
        }else if(rightButton){
            m_armSubsystem.setPulleyPower(-0.2);
        }else{
            m_armSubsystem.setPulleyPower(0);
        }

        if(leftBumper){
            m_armSubsystem.m_manipulatorSubsystem.intakeCone();
        }else if(leftTrigger){
            m_armSubsystem.m_manipulatorSubsystem.intakeCone();
        }else if(rightTrigger){
            m_armSubsystem.m_manipulatorSubsystem.smartDrop();
        }else{
            m_armSubsystem.m_manipulatorSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
