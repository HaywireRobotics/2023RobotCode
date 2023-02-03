// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoDriveState;
import frc.robot.commands.AutoDriveToTarget;
import frc.robot.commands.AutoTestDrivetrain;
import frc.robot.commands.AutoTestModuleCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DefaultElevatorCommand;
import frc.robot.commands.DefaultManipulatorCommand;
import frc.robot.commands.TestModuleCommand;
import frc.robot.networktables.DriveOdometryTable;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSybsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.TestModuleSubsystem;
import frc.robot.wrappers.Camera;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DrivetrainSubsystem m_drivetrainSubsystem = new DrivetrainSubsystem();
  // private final ElevatorSybsystem m_elevatorSybsystem = new ElevatorSybsystem();
  // private final ManipulatorSubsystem m_manipulatorSubsystem = new ManipulatorSubsystem();

  private final CommandXboxController m_controller = new CommandXboxController(0);

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

    // m_elevatorSybsystem.setDefaultCommand(new DefaultElevatorCommand(m_elevatorSybsystem, m_controller));
    // m_elevatorSybsystem.home();

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
    // Back button zeros the gyroscope
    m_controller.a().onTrue(new InstantCommand(m_drivetrainSubsystem::resetGyroscope));
    m_controller.x().whileTrue(new AutoDriveToTarget(m_drivetrainSubsystem, new Pose2d(new Translation2d(1, 1), new Rotation2d(0))));
    m_controller.b().onTrue(new InstantCommand(m_drivetrainSubsystem::resetPose));
    // new Trigger(m_controller::getBackButton).onTrue(getAutonomousCommand())(m_drivetrainSubsystem::resetGyroscope);
  
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
}
