package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.util.Statics;

public class ManualArmBindings{

    private final ArmSubsystem m_armSubsystem;
    private final CommandXboxController m_manipulator_controller;

    public ManualArmBindings(ArmSubsystem subsystem, CommandXboxController manipulatorController){
        m_armSubsystem = subsystem;
        m_manipulator_controller = manipulatorController;

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_manipulator_controller.rightBumper().whileTrue(rawManipulatorUp());
        m_manipulator_controller.rightTrigger().whileTrue(rawManipulatorDown());

        m_manipulator_controller.leftBumper().whileTrue(m_armSubsystem.m_manipulatorSubsystem.dropCommand());
        m_manipulator_controller.leftTrigger().whileTrue(m_armSubsystem.m_manipulatorSubsystem.intakeCommand());
        // m_manipulator_controller.button(3).whileTrue(m_armSubsystem.m_manipulatorSubsystem.shootCommand());

        m_manipulator_controller.rightStick().whileTrue(pulleyJoystickCommand());
        m_manipulator_controller.leftStick().whileTrue(elevatorJoystickCommand());
    }
    
    private void elevatorJoystick(){
        double yAxis2 = m_manipulator_controller.getLeftY();
        // if(yAxis2 < 0.05 && yAxis2 > -0.05) yAxis2 = 0;
        yAxis2 = Statics.applyDeadband(yAxis2, 0.05);
        m_armSubsystem.setElevatorPower(yAxis2);
    }
    private Command elevatorJoystickCommand(){
        return Commands.startEnd(this::elevatorJoystick, this::stabilizeArm, m_armSubsystem.m_elevatorSubsystem).withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
    private void pulleyJoystick(){
        double yAxis1 = m_manipulator_controller.getRightY();
        // if(yAxis1 < 0.05 && yAxis1 > -0.05) yAxis1 = 0;
        yAxis1 = Statics.applyDeadband(yAxis1, 0.05);
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
