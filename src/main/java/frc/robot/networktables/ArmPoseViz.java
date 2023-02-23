package frc.robot.networktables;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.subsystems.ArmSubsystem;

public class ArmPoseViz {


    private final ArmSubsystem m_armSubsystem;

    private final Mechanism2d mechanism2d;
    private final MechanismRoot2d armRoot;
    private final MechanismLigament2d arm;
    private final MechanismLigament2d manipulator;

    private final MechanismRoot2d elevatorRoot;
    private final MechanismLigament2d elevator;

    public ArmPoseViz(ArmSubsystem subsystem){
        this.m_armSubsystem = subsystem;

        double screenWidth = 72;
        double screenHeight = 72;
        double robotYOffset = 10;

        mechanism2d = new Mechanism2d(screenWidth, screenHeight);
        armRoot = mechanism2d.getRoot("armRoot", screenWidth/2+20, robotYOffset);
        armRoot.setPosition(0, 0);
        MechanismLigament2d arch =  armRoot.append(new MechanismLigament2d("ArchRear", 36, 90));
        arm = arch.append(new MechanismLigament2d("Arm", 36, 90));
        manipulator = arm.append(new MechanismLigament2d("Intake", 36, 0));


        elevatorRoot = mechanism2d.getRoot("elevatorRoot", screenWidth/2-20, robotYOffset);
        elevator = elevatorRoot.append(new MechanismLigament2d("Elevator", 36, 90));

        arch.setColor(new Color8Bit(Color.kWhite));
        arm.setColor(new Color8Bit(Color.kCyan));
        manipulator.setColor(new Color8Bit(Color.kDarkGreen));
        elevator.setColor(new Color8Bit(Color.kYellow));

        SmartDashboard.putData("Arm Pose", mechanism2d);
        
    }

    public void update(){
        arm.setAngle(m_armSubsystem.getArmAngle());
        arm.setLength(m_armSubsystem.getArmLength());
        manipulator.setAngle(m_armSubsystem.getManipulatorHingeAngle());
        elevator.setLength(m_armSubsystem.getElevatorPosition());
    }
}
