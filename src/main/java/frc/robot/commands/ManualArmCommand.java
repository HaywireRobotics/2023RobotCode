package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;

public class ManualArmCommand extends CommandBase {

    private final ArmSubsystem m_armSubsystem;
    private final CommandXboxController m_manipulator_controller;
    private final CommandXboxController m_drive_controller;

    private boolean hingePIDEnabled = true;
    private boolean armPIDEnabled = true;

    public ManualArmCommand(ArmSubsystem subsystem, CommandXboxController xbox, CommandXboxController manipulatorController){
        m_armSubsystem = subsystem;
        m_manipulator_controller = manipulatorController;
        m_drive_controller = xbox;

        addRequirements(subsystem);

        configureButtonBindings();
    }

    private void configureButtonBindings() {
        // m_rightJoystick.button(3).whileTrue(rawManipulatorUp());
        // m_rightJoystick.button(2).whileTrue(rawManipulatorDown());

        // m_leftJoystick.button(4).whileTrue(m_armSubsystem.m_manipulatorSubsystem.intakeCommand());
        // m_leftJoystick.button(5).whileTrue(m_armSubsystem.m_manipulatorSubsystem.dropCommand());

        // m_leftJoystick.button(1).onFalse(new InstantCommand(this::stabilizeArm));
        // m_rightJoystick.button(1).onFalse(new InstantCommand(this::stabilizeArm));

        m_manipulator_controller.rightBumper().whileTrue(rawManipulatorUp());
        // m_manipulator_controller.rightBumper().onFalse(new InstantCommand(this::stabilizeManipulator));
        m_manipulator_controller.rightTrigger().whileTrue(rawManipulatorDown());
        // m_manipulator_controller.rightTrigger().onFalse(new InstantCommand(this::stabilizeManipulator));

        m_manipulator_controller.leftBumper().whileTrue(m_armSubsystem.m_manipulatorSubsystem.dropCommand());
        m_manipulator_controller.leftTrigger().whileTrue(m_armSubsystem.m_manipulatorSubsystem.intakeCommand());

        m_manipulator_controller.rightStick().onFalse(new InstantCommand(this::stabilizeArm));
        m_manipulator_controller.leftStick().onFalse(new InstantCommand(this::stabilizeArm));
    }
    @Override
    public void execute() {
        if (hingePIDEnabled){
            m_armSubsystem.m_manipulatorSubsystem.updateHingePID();
        }
        if (armPIDEnabled){
            m_armSubsystem.m_elevatorSubsystem.updatePID();
            m_armSubsystem.m_pulleySubsystem.updatePID();
        }

        boolean isManual = false;
        if (m_manipulator_controller.rightStick().getAsBoolean()) {
            pulleyJoystick();
            isManual = true;
        }

        if (m_manipulator_controller.leftStick().getAsBoolean()) {
            elevatorJoystick();
            isManual = true;
        }

        if (m_manipulator_controller.rightBumper().getAsBoolean() || m_manipulator_controller.rightTrigger().getAsBoolean()) {
            isManual = true;
        }

        if (!m_armSubsystem.isPathFollowing && !isManual) {
            armPIDEnabled = true;
            hingePIDEnabled = true;
        }
    }
    // public Command elevatorJoystickCommand(){
    //     return new RunCommand(this::elevatorJoystick , m_armSubsystem.m_pulleySubsystem).andThen(
    //             new InstantCommand(()-> m_armSubsystem.setPulleyPower(0), m_armSubsystem.m_pulleySubsystem)
    //         ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    // }
    private void elevatorJoystick(){
        double yAxis2 = m_manipulator_controller.getLeftY();
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
        double yAxis1 = m_manipulator_controller.getRightY();
        if(yAxis1 < 0.05 && yAxis1 > -0.05) yAxis1 = 0;
        m_armSubsystem.setPulleyPower(-yAxis1);

        armPIDEnabled = false;
    }

    private Command rawManipulatorUp(){
        // return Commands.startEnd(m_armSubsystem.m_manipulatorSubsystem.rawUpCommand().alongWith(new InstantCommand(() -> {hingePIDEnabled = false;})),
        //                         stablizeManipulator());
        // return (m_armSubsystem.m_manipulatorSubsystem.rawUpCommand().alongWith(new InstantCommand(() -> {hingePIDEnabled = false;}))
        return Commands.startEnd(this::rawManipulatorUpRunnable, this::stabilizeManipulator, m_armSubsystem.m_manipulatorSubsystem);
    }
    private void rawManipulatorUpRunnable() {
        m_armSubsystem.m_manipulatorSubsystem.rawUp();
        hingePIDEnabled = false;
    }
    private Command rawManipulatorDown(){
        // return Commands.(m_armSubsystem.m_manipulatorSubsystem.rawDownCommand().alongWith(new InstantCommand(() -> {hingePIDEnabled = false;})),
        //                         stablizeManipulator(),
        //                         m_armSubsystem);
        return Commands.startEnd(this::rawManipulatorDownRunnable, this::stabilizeManipulator, m_armSubsystem.m_manipulatorSubsystem);
    }
    private void rawManipulatorDownRunnable() {
        m_armSubsystem.m_manipulatorSubsystem.rawDown();
        hingePIDEnabled = false;
    }
    private void stabilizeManipulator() {
        m_armSubsystem.stabilizeManipulator();
        hingePIDEnabled = true;
    }
    private void stabilizeArm() {
        m_armSubsystem.stabilizeArm();
        armPIDEnabled = true;
    }
    private void stabilizeAll() {
        stabilizeManipulator();
        stabilizeArm();
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
        return m_armSubsystem.m_manipulatorSubsystem.intakeCommand().alongWith(new InstantCommand(() -> {
            hingePIDEnabled = true;
            m_armSubsystem.m_manipulatorSubsystem.setHingeTarget(70);
        }));
    }
    private Command pidIntakeCube(){
        return m_armSubsystem.m_manipulatorSubsystem.intakeCommand().alongWith(new InstantCommand(() -> {
            hingePIDEnabled = true;
            m_armSubsystem.m_manipulatorSubsystem.setHingeTarget(70);
        }));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}