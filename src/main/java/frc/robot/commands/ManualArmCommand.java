package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;

public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;
    private final CommandJoystick m_rightJoystick;
    private final CommandJoystick m_leftJoystick;
    private final CommandXboxController m_drive_controller;

    private boolean hingePIDEnabled = false;
    private boolean armPIDEnabled = false;

    public ManualArmCommand(ArmSubsystem subsystem, CommandXboxController xbox, CommandJoystick joystick1, CommandJoystick joystick2){
        m_armSubsystem = subsystem;
        m_rightJoystick = joystick1;
        m_leftJoystick = joystick2;
        m_drive_controller = xbox;

        addRequirements(subsystem);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        m_rightJoystick.button(3).whileTrue(rawManipulatorUp());
        m_rightJoystick.button(2).whileTrue(rawManipulatorDown());

        m_leftJoystick.button(4).whileTrue(m_armSubsystem.m_manipulatorSubsystem.intakeConeCommand());
        m_leftJoystick.button(5).whileTrue(m_armSubsystem.m_manipulatorSubsystem.intakeCubeCommand());

        m_leftJoystick.button(3).whileTrue(m_armSubsystem.m_manipulatorSubsystem.shootCubeCommand());
    }
    @Override
    public void execute() {
        if(hingePIDEnabled){
            m_armSubsystem.m_manipulatorSubsystem.updateHingePID();
        }
        if (armPIDEnabled){
            m_armSubsystem.m_elevatorSubsystem.updatePID();
            m_armSubsystem.m_pulleySubsystem.updatePID();
        }
        if (m_rightJoystick.button(1).getAsBoolean()) {
            pulleyJoystick();
        } else if (!armPIDEnabled){
            m_armSubsystem.setPulleyPower(0);
        }

        if (m_leftJoystick.button(1).getAsBoolean()) {
            elevatorJoystick();
        } else if (!armPIDEnabled){
            m_armSubsystem.setElevatorPower(0);
        }
    }
    // public Command elevatorJoystickCommand(){
    //     return new RunCommand(this::elevatorJoystick , m_armSubsystem.m_pulleySubsystem).andThen(
    //             new InstantCommand(()-> m_armSubsystem.setPulleyPower(0), m_armSubsystem.m_pulleySubsystem)
    //         ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    // }
    private void elevatorJoystick(){
        double yAxis2 = m_leftJoystick.getY();
        if(yAxis2 < 0.05 && yAxis2 > -0.05) yAxis2 = 0;
        m_armSubsystem.setElevatorPower(yAxis2);

        armPIDEnabled = false;
    }
    // public Command pulleyJoystickCommand(){
    //     return new RunCommand(this::pulleyJoystick , m_armSubsystem.m_pulleySubsystem).andThen(
    //             new InstantCommand(()-> m_armSubsystem.setPulleyPower(0), m_armSubsystem.m_pulleySubsystem)
    //         ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    // }
    private void pulleyJoystick(){
        double yAxis1 = m_rightJoystick.getY();
        if(yAxis1 < 0.05 && yAxis1 > -0.05) yAxis1 = 0;
        m_armSubsystem.setPulleyPower(-yAxis1);

        armPIDEnabled = false;
    }

    private Command rawManipulatorUp(){
        // return Commands.startEnd(m_armSubsystem.m_manipulatorSubsystem.rawUpCommand().alongWith(new InstantCommand(() -> {hingePIDEnabled = false;})),
        //                         stablizeManipulator());
        // return (m_armSubsystem.m_manipulatorSubsystem.rawUpCommand().alongWith(new InstantCommand(() -> {hingePIDEnabled = false;}))
        return Commands.startEnd(this::rawManipulatorUpRunnable, this::stablizeManipulator, m_armSubsystem.m_manipulatorSubsystem);
    }
    private void rawManipulatorUpRunnable() {
        m_armSubsystem.m_manipulatorSubsystem.rawUp();
        hingePIDEnabled = false;
    }
    private Command rawManipulatorDown(){
        // return Commands.(m_armSubsystem.m_manipulatorSubsystem.rawDownCommand().alongWith(new InstantCommand(() -> {hingePIDEnabled = false;})),
        //                         stablizeManipulator(),
        //                         m_armSubsystem);
        return Commands.startEnd(this::rawManipulatorDownRunnable, this::stablizeManipulator, m_armSubsystem.m_manipulatorSubsystem);
    }
    private void rawManipulatorDownRunnable() {
        m_armSubsystem.m_manipulatorSubsystem.rawDown();
        hingePIDEnabled = false;
    }
    private void stablizeManipulator() {
        double currentAngle = m_armSubsystem.getManipulatorHingeAngle();
        m_armSubsystem.m_manipulatorSubsystem.setHingeTarget(currentAngle);
        hingePIDEnabled = true;
    }
    private Command pidManipulatorUp(){
        return m_armSubsystem.m_manipulatorSubsystem.setHingeUpCommand().alongWith(new InstantCommand(() -> {hingePIDEnabled = true;}));
    }
    private Command pidStowArm(){
        return m_armSubsystem.collapseArmCommand().andThen(new InstantCommand(() -> {armPIDEnabled = true;}));
    }
    private Command pidGroundIntakeCube(){
        return new InstantCommand(this::pidGroundIntakeCubeSetpoint);
    }
    private void pidGroundIntakeCubeSetpoint(){
        // Top of cone at 50"
        m_armSubsystem.setPulleyTarget(4.5);
        m_armSubsystem.setElevatorTarget(1.2);
        armPIDEnabled = true;
        hingePIDEnabled = true;
        m_armSubsystem.setManipulatorHingeTarget(75);
        
    }
    private Command pidSubstationArm(){
        return new InstantCommand(this::pidSubstationSetpoint);
    }
    private void pidSubstationSetpoint(){
        // Top of cone at 50"
        m_armSubsystem.setPulleyTarget(24);
        m_armSubsystem.setElevatorTarget(25.5);
        armPIDEnabled = true;
        hingePIDEnabled = true;
        m_armSubsystem.setManipulatorHingeTarget(123);
        
    }
    private Command pidManipulatorDown(){
        return m_armSubsystem.m_manipulatorSubsystem.setHingeDownCommand().alongWith(new InstantCommand(() -> {hingePIDEnabled = true;}));
    }

    private Command pidIntakeCone(){
        return m_armSubsystem.m_manipulatorSubsystem.intakeConeCommand().alongWith(new InstantCommand(() -> {
            hingePIDEnabled = true;
            m_armSubsystem.m_manipulatorSubsystem.setHingeTarget(70);
        }));
    }
    private Command pidIntakeCube(){
        return m_armSubsystem.m_manipulatorSubsystem.intakeConeCommand().alongWith(new InstantCommand(() -> {
            hingePIDEnabled = true;
            m_armSubsystem.m_manipulatorSubsystem.setHingeTarget(70);
        }));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
