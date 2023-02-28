package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;
    private final CommandJoystick m_aux_controller1;
    private final CommandJoystick m_aux_controller2;
    private final CommandXboxController m_drive_controller;

    public ManualArmCommand(ArmSubsystem subsystem, CommandXboxController xbox, CommandJoystick joystick1, CommandJoystick joystick2){
        m_armSubsystem = subsystem;
        m_aux_controller1 = joystick1;
        m_aux_controller2 = joystick2;
        m_drive_controller = xbox;

        addRequirements(subsystem);

        configureButtonBindings();
    }

    private void configureButtonBindings(){
        // m_controller.button(0).whileTrue(m_armSubsystem.m_pulleySubsystem.extendCommand());
        // m_controller.button(1).whileTrue(m_armSubsystem.m_pulleySubsystem.retractCommand());

        m_aux_controller1.button(3).whileTrue(m_armSubsystem.m_manipulatorSubsystem.rawUpCommand());
        m_aux_controller1.button(2).whileTrue(m_armSubsystem.m_manipulatorSubsystem.rawDownCommand());

        // m_controller1.button(3).onTrue(m_armSubsystem.m_manipulatorSubsystem.setHingeUpCommand());
        // m_controller1.button(2).onTrue(m_armSubsystem.m_manipulatorSubsystem.setHingeDownCommand());

        // m_aux_controller1.button(4).onTrue(m_armSubsystem.m_manipulatorSubsystem.intakeConeCommand());
        // m_aux_controller1.button(5).onTrue(m_armSubsystem.m_manipulatorSubsystem.intakeCubeCommand());
        // m_controller.button(1).onTrue(m_armSubsystem.m_manipulatorSubsystem.smartDropCommand());

        m_drive_controller.leftBumper().whileTrue(m_armSubsystem.m_elevatorSubsystem.raiseArm());
        m_drive_controller.leftBumper().whileTrue(m_armSubsystem.m_elevatorSubsystem.raiseArm());
    }
    @Override
    public void execute() {

        double yAxis1 = m_aux_controller1.getY();
        if(yAxis1 < 0.05 && yAxis1 > -0.05) yAxis1 = 0;

        double yAxis2 = m_aux_controller2.getY();
        if(yAxis2 < 0.05 && yAxis2 > -0.05) yAxis2 = 0;

        boolean extensionButton = m_aux_controller1.button(1).getAsBoolean();
        boolean elevatorButton = m_aux_controller2.button(1).getAsBoolean();

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
    // public Command elevatorJoystickCommand(){
        
    //     return new RunCommand(() -> {
    //         double yAxis1 = m_aux_controller1.getY();
    //         if(yAxis1 < 0.05 && yAxis1 > -0.05) yAxis1 = 0;
    //         m_armSubsystem.setPulleyPower(yAxis1);
    //         } , m_armSubsystem.m_pulleySubsystem).andThen(()-> {m_armSubsystem.setPulleyPower(0)})
    // }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
