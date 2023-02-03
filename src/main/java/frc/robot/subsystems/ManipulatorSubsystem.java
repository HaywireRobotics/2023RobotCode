package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ManipulatorSubsystem extends SubsystemBase{
    private final NEO rollerMotor;

    private final double coneMotorSpeed = 0.1;
    private final double cubeMotorSpeed = -0.1;

    public ManipulatorSubsystem(){
        rollerMotor = new NEO(Constants.MANIPULATOR_ROLLER_MOTOR, IdleMode.kBrake);
    }

    public void intakeCone(){
        rollerMotor.set(coneMotorSpeed);
    }

    public void intakeCube(){
        rollerMotor.set(cubeMotorSpeed);
    }

    public void stop(){
        rollerMotor.set(0);
    }
}
