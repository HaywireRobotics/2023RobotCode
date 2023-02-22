// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.AutoDriveState;
import frc.robot.commands.PositionAprilTag;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.networktables.DriveOdometryTable;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.wrappers.Camera;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();

  private final ElevatorSubsystem m_elevatorSubsystem = new ElevatorSubsystem();
  private final ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem();
  private final PulleySubsystem m_pulleySubsystem = new PulleySubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_pulleySubsystem, m_elevatorSubsystem, m_manipulatorSubsystem);

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final Joystick m_leftJoystick = new Joystick(1);
  private final Joystick m_right_Joystick = new Joystick(2);

  private final NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();
  private final DriveOdometryTable m_drivetrainTable = new DriveOdometryTable(m_networkTable);

  public final Camera m_camera = new Camera(m_networkTable);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Set up the default command for the drivetrain.
    // The controls are for field-oriented driving:
    // Left stick Y axis -> forward and backwards movement
    // Left stick X axis -> left and right movement
    // Right stick X axis -> rotation
    m_drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(m_drivetrainSubsystem, m_controller));

    m_armSubsystem.setDefaultCommand(new ManualArmCommand(m_armSubsystem, m_controller));

    // m_elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(m_elevatorSubsystem, m_controller));
    // m_elevatorSubsystem.home();

    // m_manipulatorSubsystem.setDefaultCommand(new DefaultManipulatorCommand(m_manipulatorSubsystem, m_controller));
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // manages drive mode stuff
    // A -> reset gyroscope, ie, 0 is now where you are pointing
    // B -> reset pose, ie, you are now at (0,0)
    // Y -> toggle field-centric, ie, if you hit it it drives like a drone
    m_controller.a().onTrue(new InstantCommand(m_drivetrainSubsystem::resetGyroscope));
    m_controller.b().onTrue(new InstantCommand(m_drivetrainSubsystem::resetPose));
    m_controller.y().onTrue(new InstantCommand(m_drivetrainSubsystem::toggleFieldCentricDrive));

    // AutoDriveToTarget stuff aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
    // m_controller.x().whileTrue(new AutoDriveToTarget(m_drivetrainSubsystem, new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0))));
    m_controller.x().whileTrue(new PositionAprilTag(m_drivetrainSubsystem, m_camera, 1, 0, 0));
    // drive using D-pad
    // m_controller.povDown().whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setAllToState(new SwerveModuleState(300, Rotation2d.fromDegrees(0))), m_drivetrainSubsystem));
    // m_controller.povUp().whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setAllToState(new SwerveModuleState(300, Rotation2d.fromDegrees(180))), m_drivetrainSubsystem));
    // m_controller.povLeft().whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setAllToState(new SwerveModuleState(300, Rotation2d.fromDegrees(270))), m_drivetrainSubsystem));
    // m_controller.povRight().whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setAllToState(new SwerveModuleState(300, Rotation2d.fromDegrees(90))), m_drivetrainSubsystem));  
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new AutoTestModuleCommand(m_drivetrainSubsystem.frontLeftState);
    // return new AutoTestDrivetrain(m_drivetrainSubsystem);
    return new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(0, Rotation2d.fromDegrees(0)));
    // return new InstantCommand();
    
  }

  public void updateNetworkTables(){
    m_drivetrainTable.publishPose(m_drivetrainSubsystem.getPose());
  }
  public void resetOdometry(){
    m_drivetrainSubsystem.resetPose();
  }

  public void disable(){
    m_drivetrainSubsystem.disable();
  }
  public void enable(){
    m_drivetrainSubsystem.enable();
  }
}
