// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AutoArmToSetpoint;
import frc.robot.commands.PositionAprilTag;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualBalanceDrive;
import frc.robot.networktables.ArmPoseViz;
import frc.robot.networktables.ArmTable;
import frc.robot.networktables.DriveOdometryTable;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.ManipulatorSubsystem;
import frc.robot.subsystems.PulleySubsystem;
import frc.robot.util.ArmAutoPath;
import frc.robot.wrappers.AutoCommands;
import frc.robot.wrappers.Camera;
import frc.robot.wrappers.DriverCamera;
import frc.robot.wrappers.LEDs;

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

  private final ArmPoseViz m_armPoseViz = new ArmPoseViz(m_armSubsystem);

  private final CommandXboxController m_controller = new CommandXboxController(0);
  private final CommandJoystick m_auxJoystick1 = new CommandJoystick(1);
  private final CommandJoystick m_auxJoystick2 = new CommandJoystick(2);
  // private final Joystick m_right_Joystick = new Joystick(2);

  private final NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();
  private final DriveOdometryTable m_drivetrainTable = new DriveOdometryTable(m_networkTable, m_drivetrainSubsystem);
  private final ArmTable m_armTable = new ArmTable(m_networkTable, m_armSubsystem);

  SendableChooser<Command> m_auto_chooser;

  // private final Command[] m_auto_commands = {
  //   new AutoDriveToTarget(m_drivetrainSubsystem, new Pose2d(new Translation2d(0.0, 2.0), new Rotation2d(0))),
  //   m_manipulatorSubsystem.shootCubeCommand(),
  //   new AutoDriveState(m_drivetrainSubsystem, new SwerveModuleState(500.0, Rotation2d.fromDegrees(0.0))).withTimeout(2)
  // };
  private final AutoCommands m_autoCommands = new AutoCommands(m_drivetrainSubsystem, m_armSubsystem);

  public final Camera m_limelight = new Camera(m_networkTable, "OV5647");
  public final DriverCamera m_driverCamera = new DriverCamera("Driver Camera", 0);

  public final LEDs m_leds = new LEDs(9);
  public Color m_ledColor = Color.kOrange;


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

    m_armSubsystem.setDefaultCommand(new ManualArmCommand(m_armSubsystem, m_controller, m_auxJoystick1, m_auxJoystick2));
    // m_armSubsystem.setDefaultCommand(new AutoArmToSetpoint(m_armSubsystem, Constants.ArmSetpointPaths.STOWED));

    m_auto_chooser = new SendableChooser<>();
    m_auto_chooser.setDefaultOption("Cube Auto", m_autoCommands.NoDriveCubeCommand());
    m_auto_chooser.addOption("Leave Community Drop Cube", m_autoCommands.LeaveCommunityCubeCommand());
    m_auto_chooser.addOption("Leave Community No Cube", m_autoCommands.LeaveCommunityNoCubeCommand());
    m_auto_chooser.addOption("Dock Drop Cube", m_autoCommands.DockCubeCommand());
    m_auto_chooser.addOption("Dock No Cube", m_autoCommands.DockNoCubeCommand());
    m_auto_chooser.addOption("NO Auto", new InstantCommand(() -> {m_drivetrainSubsystem.setGyroOffset(180);}));
    m_auto_chooser.addOption("testAuto", m_autoCommands.testAuto());
    m_auto_chooser.addOption("testTrajectory", m_autoCommands.testTrajectory());
    // m_auto_chooser.addOption("Drive Auto", m_auto_commands[2]);
    // m_elevatorSubsystem.setDefaultCommand(new DefaultElevatorCommand(m_elevatorSubsystem, m_controller));
    // m_elevatorSubsystem.home();

    SmartDashboard.putData(m_auto_chooser);

    m_leds.setSolid(m_ledColor);

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
    // m_controller.a().onTrue(new InstantCommand(m_drivetrainSubsystem::resetGyroscope));
    // m_controller.b().onTrue(new InstantCommand(m_drivetrainSubsystem::resetPose));
    m_controller.back().onTrue(new InstantCommand(m_drivetrainSubsystem::toggleFieldCentricDrive));
    m_controller.start().onTrue(new InstantCommand(m_drivetrainSubsystem::resetPose));

    // m_controller.y().whileTrue(smartSetpointCommand(Constants.ScoreRows.HIGH));
    // m_controller.b().whileTrue(smartSetpointCommand(Constants.ScoreRows.MID));
    // m_controller.a().whileTrue(smartSetpointCommand(Constants.ScoreRows.LOW));

    m_controller.y().whileTrue(adaptiveSetpointCommand(Constants.ScorePositions.CONE_HIGH));
    m_controller.b().whileTrue(adaptiveSetpointCommand(Constants.ScorePositions.CONE_MID));
    m_controller.a().whileTrue(adaptiveSetpointCommand(Constants.ScorePositions.GROUND));
    m_controller.x().whileTrue(adaptiveSetpointCommand(Constants.ScorePositions.SUBSTATION));
    m_controller.rightBumper().whileTrue(adaptiveSetpointCommand(Constants.ScorePositions.STOW));

    m_auxJoystick1.button(4).onTrue(adaptiveSetpointCommand(Constants.ScorePositions.CONE_HIGH));
    m_auxJoystick1.button(5).onTrue(adaptiveSetpointCommand(Constants.ScorePositions.CONE_MID));
    m_auxJoystick2.button(3).onTrue(adaptiveSetpointCommand(Constants.ScorePositions.SUBSTATION));


    // m_controller.leftStick().toggleOnTrue(new ManualBalanceDrive(m_drivetrainSubsystem, m_controller));
    m_controller.leftBumper().whileTrue(new ManualBalanceDrive(m_drivetrainSubsystem, m_controller));
    m_controller.rightTrigger().whileTrue(new PositionAprilTag(m_drivetrainSubsystem, m_limelight, 1.4, 0, 0, true));

    m_controller.rightStick().onTrue(new InstantCommand(() -> {
      if (m_ledColor == Color.kOrange) {
        m_ledColor = Color.kPurple;
      } else {
        m_ledColor = Color.kOrange;
      }
      m_leds.setSolid(m_ledColor);
    }));

    // m_controller.y().whileTrue(new AlignSubstationAprilTag(m_drivetrainSubsystem, m_limelight));

    // AutoDriveToTarget stuff aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa
    // m_controller.x().whileTrue(new AutoDriveToTarget(m_drivetrainSubsystem, new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0))));
    // m_controller.x().whileTrue(new PositionAprilTag(m_drivetrainSubsystem, m_camera, 1, 0, 0));
    // drive using D-pad
    // m_controller.povDown().whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setAllToState(new SwerveModuleState(300, Rotation2d.fromDegrees(0))), m_drivetrainSubsystem));
    // m_controller.povUp().whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setAllToState(new SwerveModuleState(300, Rotation2d.fromDegrees(180))), m_drivetrainSubsystem));
    // m_controller.povLeft().whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setAllToState(new SwerveModuleState(300, Rotation2d.fromDegrees(270))), m_drivetrainSubsystem));
    // m_controller.povRight().whileTrue(new RunCommand(() -> m_drivetrainSubsystem.setAllToState(new SwerveModuleState(300, Rotation2d.fromDegrees(90))), m_drivetrainSubsystem));  
    
    // Uncomment to test auto scoring.
    // m_controller.leftBumper().whileTrue(AutoScore.autoScoreCommand(m_drivetrainSubsystem, m_armSubsystem, Constants.Alliances.BLUE, Constants.ScorePositions.CONE_HEIGH, 0));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // return new AutoTestModuleCommand(m_drivetrainSubsystem.frontLeftState);
    // return new AutoTestDrivetrain(m_drivetrainSubsystem);
    return m_auto_chooser.getSelected();
    // return new InstantCommand();
    
  }
  public Command smartSetpointCommand(Constants.ScoreRows row){
    Constants.GamePieces gamePiece = m_manipulatorSubsystem.getGamePiece();
    ArmAutoPath path;
    Constants.ScorePositions scorePosition = Constants.ScorePositions.GROUND;

    if(row == Constants.ScoreRows.HIGH){
      if(gamePiece == Constants.GamePieces.CUBE){
        scorePosition = Constants.ScorePositions.CUBE_HIGH;
      } else {
        scorePosition = Constants.ScorePositions.CONE_HIGH;
      }
    } else if (row == Constants.ScoreRows.MID){
      if(gamePiece == Constants.GamePieces.CUBE){
        scorePosition = Constants.ScorePositions.CUBE_MID;
      } else {
        scorePosition = Constants.ScorePositions.CONE_MID;
      }
    }else{
      scorePosition = Constants.ScorePositions.GROUND;
    }

    path = Constants.ArmSetpointPaths.getPathForScorePosition(scorePosition);
    return new AutoArmToSetpoint(m_armSubsystem, path);
  }
  public Command adaptiveSetpointCommand(Constants.ScorePositions scorePosition){
    ArmAutoPath path;
    double distanceToHigh = Constants.ArmSetpoints.CONE_HIGH.armPosition.subtract(m_armSubsystem.getManipulator2dPosition()).magnitude();
    if(scorePosition == Constants.ScorePositions.CONE_MID && distanceToHigh < 5){
      path = Constants.ArmSetpointPaths.CONE_HIGH_TO_MID;
    }else{
      path = Constants.ArmSetpointPaths.getPathForScorePosition(scorePosition);
    }
    return new AutoArmToSetpoint(m_armSubsystem, path);
  }
  public void updateCamera(){
    m_limelight.update();
  }
  public void mergeCameraPose(){
    m_drivetrainSubsystem.mergeCameraPose(m_limelight.getRobotPose2d(), m_limelight.getPoseConfidence()/10);//, m_limelight.getPoseConfidence());
  }

  public void updateLEDs(){
    // m_leds.setSolid(Color.kYellow);
    // if(m_armSubsystem.isAllAtSetpoint()){
    //   m_leds.setSolid(Color.kGreen);
    // } if(m_limelight.getPoseConfidence() < 0.25){
    //   m_leds.setSolid(Color.kRed);
    // } if(m_limelight.getPoseConfidence() < 0.75){
    //   Color[] c = {Color.kRed, Color.kGreen};
    //   m_leds.setCycle(c, 0.5);
    // }else {
    //   m_leds.setSolid(Color.kYellow);
    // }
    m_leds.update();
  }

  public void updateNetworkTables(){
    m_drivetrainTable.publishData();
    m_armPoseViz.update();
    m_armTable.publishData();
  }
  public void resetOdometry(){
    m_drivetrainSubsystem.resetPose();
  }
  public void zeroGyro(){
    m_drivetrainSubsystem.resetGyroscope();
  }
  // public void resetGyroFromMag(){
  //   m_drivetrainSubsystem.gyroFromMag(Constants.fieldHeadingMag);
  // }

  public void resetEncoders(){
    m_armSubsystem.m_elevatorSubsystem.resetEncoder();
    m_armSubsystem.m_manipulatorSubsystem.resetEncoder();
    m_armSubsystem.m_pulleySubsystem.resetEncoder(0.0);
  }

  public void disable(){
    m_drivetrainSubsystem.disable();
    m_armSubsystem.disable();
  }
  public void enable(){
    m_drivetrainSubsystem.enable();
    m_armSubsystem.enable();
    m_drivetrainSubsystem.resetGyroscope();
    m_drivetrainSubsystem.resetPose();
    // m_armSubsystem.resetEncoders();
  }
}
