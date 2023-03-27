package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmBindings{

    private final ArmSubsystem m_armSubsystem;
    private final CommandJoystick m_rightJoystick;
    private final CommandJoystick m_leftJoystick;
    private final CommandXboxController m_drive_controller;

    public ManualArmBindings(ArmSubsystem subsystem, CommandXboxController xbox, CommandJoystick joystick1, CommandJoystick joystick2){
        m_armSubsystem = subsystem;
        m_rightJoystick = joystick1;
        m_leftJoystick = joystick2;
        m_drive_controller = xbox;

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_rightJoystick.button(3).whileTrue(rawManipulatorUp());
        m_rightJoystick.button(2).whileTrue(rawManipulatorDown());

        m_leftJoystick.button(4).whileTrue(m_armSubsystem.m_manipulatorSubsystem.intakeCommand());
        m_leftJoystick.button(5).whileTrue(m_armSubsystem.m_manipulatorSubsystem.dropCommand());
        m_leftJoystick.button(3).whileTrue(m_armSubsystem.m_manipulatorSubsystem.shootCommand());

        m_rightJoystick.button(1).whileTrue(pulleyJoystickCommand());
        m_leftJoystick.button(1).whileTrue(elevatorJoystickCommand());
    }
    
    private void elevatorJoystick(){
        double yAxis2 = m_leftJoystick.getY();
        if(yAxis2 < 0.05 && yAxis2 > -0.05) yAxis2 = 0;
        m_armSubsystem.setElevatorPower(yAxis2);
    }
    private Command elevatorJoystickCommand(){
        return Commands.startEnd(this::elevatorJoystick, this::stabilizeArm, m_armSubsystem.m_elevatorSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    private void pulleyJoystick(){
        double yAxis1 = m_rightJoystick.getY();
        if(yAxis1 < 0.05 && yAxis1 > -0.05) yAxis1 = 0;
        m_armSubsystem.setPulleyPower(-yAxis1);
    }
    private Command pulleyJoystickCommand(){
        return Commands.startEnd(this::pulleyJoystick, this::stabilizeArm, m_armSubsystem.m_pulleySubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    private Command rawManipulatorUp(){
        return Commands.startEnd(this::rawManipulatorUpRunnable, this::stabilizeManipulator, m_armSubsystem.m_manipulatorSubsystem);
    }
    private void rawManipulatorUpRunnable() {
        m_armSubsystem.m_manipulatorSubsystem.rawUp();
    }
    private Command rawManipulatorDown(){
        return Commands.startEnd(this::rawManipulatorDownRunnable, this::stabilizeManipulator, m_armSubsystem.m_manipulatorSubsystem);
    }
    private void rawManipulatorDownRunnable() {
        m_armSubsystem.m_manipulatorSubsystem.rawDown();
    }
    private void stabilizeManipulator() {
        m_armSubsystem.stabilizeManipulator();
    }
    private void stabilizeArm() {
        m_armSubsystem.stabilizeArm();
    }
}
