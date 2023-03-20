package frc.robot.networktables;

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
        armRoot = mechanism2d.getRoot("armRoot", 20, robotYOffset);
        armRoot.setPosition(0, 0);
        MechanismLigament2d arch =  armRoot.append(new MechanismLigament2d("ArchRear", 36, 90));
        arm = arch.append(new MechanismLigament2d("Arm", 29, 90));
        manipulator = arm.append(new MechanismLigament2d("Intake", 12, 0));


        elevatorRoot = mechanism2d.getRoot("elevatorRoot", 21, robotYOffset);
        elevator = elevatorRoot.append(new MechanismLigament2d("Elevator", 29, 90));

        arch.setColor(new Color8Bit(Color.kWhite));
        arm.setColor(new Color8Bit(Color.kCyan));
        manipulator.setColor(new Color8Bit(Color.kDarkGreen));
        elevator.setColor(new Color8Bit(Color.kYellow));

        SmartDashboard.putData("Arm Pose", mechanism2d);
        
    }

    public void update(){
        arm.setAngle(m_armSubsystem.getArmAngle()-90);
        arm.setLength(m_armSubsystem.getArmLength());
        elevator.setLength(m_armSubsystem.getElevatorPosition());
        manipulator.setAngle(-m_armSubsystem.getManipulatorHingeAngle()+140);

        // manipulator gamePiece Cone: Yellow, manipulator gamePiece Cube: Purple gamePiece: None, white
        switch(m_armSubsystem.m_manipulatorSubsystem.getGamePiece()){
            case NONE:
                manipulator.setColor(new Color8Bit(Color.kWhite));
                break;
            case CUBE:
                manipulator.setColor(new Color8Bit(Color.kPurple));
                break;
            case CONE:
                manipulator.setColor(new Color8Bit(Color.kYellow));
                break;
        }
        
    }
}
