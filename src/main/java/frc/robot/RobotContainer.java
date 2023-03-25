// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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
import frc.robot.commands.AutoDriveToTarget;
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
import frc.robot.wrappers.AdvancedSetpoints;
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
    private final CommandJoystick m_rightJoystick = new CommandJoystick(1);
    private final CommandJoystick m_leftJoystick = new CommandJoystick(2);

    private final NetworkTableInstance m_networkTable = NetworkTableInstance.getDefault();
    private final DriveOdometryTable m_drivetrainTable = new DriveOdometryTable(m_networkTable, m_drivetrainSubsystem);
    private final ArmTable m_armTable = new ArmTable(m_networkTable, m_armSubsystem);

    SendableChooser<Command> m_auto_chooser;
    private final AdvancedSetpoints m_advancedSetpoints = new AdvancedSetpoints(m_drivetrainSubsystem, m_armSubsystem, m_manipulatorSubsystem);
    private final AutoCommands m_autoCommands = new AutoCommands(m_drivetrainSubsystem, m_armSubsystem, m_manipulatorSubsystem, m_advancedSetpoints);

    public final Camera m_limelight = new Camera(m_networkTable, "OV5647");
    public final DriverCamera m_driverCamera = new DriverCamera("Driver Camera", 0);

    public final LEDs m_leds = new LEDs(9);

    public Alliance alliance = Alliance.Invalid;


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

        m_armSubsystem.setDefaultCommand(new ManualArmCommand(m_armSubsystem, m_controller, m_rightJoystick, m_leftJoystick));
        // m_armSubsystem.setDefaultCommand(new AutoArmToSetpoint(m_armSubsystem, Constants.ArmSetpointPaths.STOWED));

        m_auto_chooser = new SendableChooser<>();
        m_auto_chooser.setDefaultOption("Cube Auto", m_autoCommands.NoDriveCubeCommand());
        m_auto_chooser.addOption("Leave Community Drop Cube", m_autoCommands.LeaveCommunityCubeCommand());
        m_auto_chooser.addOption("Leave Community No Cube", m_autoCommands.LeaveCommunityNoCubeCommand());
        m_auto_chooser.addOption("Dock Drop Cube", m_autoCommands.DockCubeCommand());
        m_auto_chooser.addOption("Dock No Cube", m_autoCommands.DockNoCubeCommand());
        m_auto_chooser.addOption("NO Auto", m_drivetrainSubsystem.flipGyroCommand());
        m_auto_chooser.addOption("High Cone", m_autoCommands.HighConeCommand());
        m_auto_chooser.addOption("testAuto", m_autoCommands.testAuto());
        m_auto_chooser.addOption("testTrajectory", m_autoCommands.testTrajectory());

        SmartDashboard.putData(m_auto_chooser);

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
        m_controller.povDown().onTrue(new InstantCommand(()->{m_drivetrainSubsystem.resetGyroscope();}));

        m_controller.y().whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.CONE_HIGH));
        m_controller.b().whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.CONE_MID));
        m_controller.a().whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.GROUND));
        m_controller.x().whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.SUBSTATION));
        // m_controller.x().onTrue(m_advancedSetpoints.substationCommand());
        m_controller.rightBumper().whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.STOW));

        m_rightJoystick.button(4).whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.CONE_HIGH));
        m_rightJoystick.button(5).whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.CONE_MID));
        m_leftJoystick.button(3).whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.SUBSTATION));
        m_leftJoystick.button(2).whileTrue(m_armSubsystem.adaptiveSetpointCommand(Constants.SetpointPositions.TIPPED_PICKUP));

        // m_controller.leftStick().toggleOnTrue(new ManualBalanceDrive(m_drivetrainSubsystem, m_controller));
        m_controller.leftBumper().whileTrue(new ManualBalanceDrive(m_drivetrainSubsystem, m_controller));
        m_controller.leftTrigger().whileTrue(new PositionAprilTag(m_drivetrainSubsystem, m_limelight, 1.4, 0.0));
        m_controller.rightTrigger().whileTrue(new AutoDriveToTarget(m_drivetrainSubsystem, Constants.DriveSetpoints.BlueSubstation[0]));
        m_controller.rightStick().onTrue(new InstantCommand(m_leds::toggleColor));

        m_controller.povLeft().onTrue(m_advancedSetpoints.IntakeCubeCommand());
 
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
    
    public void updateCamera(){
        m_limelight.update();
    }
    public void mergeCameraPose(){
        m_drivetrainSubsystem.mergeCameraPose(m_limelight.getRobotPose2d(), m_limelight.getPoseConfidence()/10);//, m_limelight.getPoseConfidence());
    }

    public void updateLEDs() {
        if (m_drivetrainSubsystem.aligning) {
            if (m_drivetrainSubsystem.aligned) {
                m_leds.setSolidGamePieceColor();
            } else {
                m_leds.blinkGamePieceColor();
            }
        }

        // if (m_armSubsystem.isAllAtSetpoint()) {
        //     m_leds.setSolid(Color.kGreen);
        // }

        // if(m_limelight.getPoseConfidence() < 0.25){
        //     m_leds.setSolid(Color.kRed);
        // } if(m_limelight.getPoseConfidence() < 0.75){
        //     Color[] c = {Color.kRed, Color.kGreen};
        //     m_leds.setCycle(c, 0.5);
        // }else {
        //     m_leds.setSolid(Color.kYellow);
        // }
        m_leds.update();
    }

    // needed to handle the alliance being unknown until the driverstation connects
    public void updateAlliance() {
        Alliance actualAlliance = DriverStation.getAlliance();
        if (alliance != actualAlliance) {
            switch (actualAlliance) {
                case Red:
                    alliance = Alliance.Red;
                    m_leds.setAllianceColor(Color.kRed);
                    break;

                case Blue:
                    alliance = Alliance.Blue;
                    m_leds.setAllianceColor(Color.kBlue);
                    break;
            
                default:
                    alliance = Alliance.Invalid;
                    m_leds.setAllianceColor(Color.kPurple);
                    break;
            }
        }
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
        // m_drivetrainSubsystem.resetGyroscope();
        // m_drivetrainSubsystem.resetPose();
        // m_armSubsystem.resetEncoders();
    }
}
