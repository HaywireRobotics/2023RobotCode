package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ArmSubsystem {
    private final NEO pulleyMotor;
    private final PIDController pulleyPID;

    private final NEO manipulatorHingeMotor;
    private final PIDController manipulatorController;

    private final double PULLEY_KP = 0.04;
    private final double PULLEY_KI = 0;
    private final double PULLEY_KD = 0;

    private final double MANIPULATOR_KP = 0.04;
    private final double MANIPULATOR_KI = 0;
    private final double MANIPULATOR_KD = 0;

    public ArmSubsystem(){
        pulleyMotor = new NEO(Constants.PULLEY_MOTOR, IdleMode.kBrake);
        pulleyPID = new PIDController(PULLEY_KP, PULLEY_KI, PULLEY_KD);

        manipulatorHingeMotor = new NEO(Constants.MANIPULATOR_HINGE_MOTOR, IdleMode.kBrake);
        manipulatorController = new PIDController(MANIPULATOR_KP, MANIPULATOR_KI, MANIPULATOR_KD);
    }

    public void periodic() {
        
    }
}
