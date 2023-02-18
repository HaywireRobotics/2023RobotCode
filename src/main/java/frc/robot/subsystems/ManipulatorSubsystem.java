package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Statics;
import frc.robot.wrappers.NEO;

public class ManipulatorSubsystem extends SubsystemBase{
    private final NEO rollerMotor;

    private final double coneMotorSpeed = 0.1;
    private final double cubeMotorSpeed = -0.1;

    private final NEO hingeMotor;
    private final PIDController hingePID;

    private final double MANIPULATOR_KP = 0.04;
    private final double MANIPULATOR_KI = 0;
    private final double MANIPULATOR_KD = 0;

    private final double MANIPULATOR_UP_ANGLE = 90;
    private final double MANIPULATOR_DOWN_ANGLE = 0;
    private final double MANIPULATOR_POWER_OFF_ERROR = 5;
    private final double MANIPULATOR_HINGE_MAX_POWER = 0.5;
    private final double MANIPULATOR_HINGE_MIN_POWER = -0.2;

    public ManipulatorSubsystem(){
        rollerMotor = new NEO(Constants.MANIPULATOR_ROLLER_MOTOR, IdleMode.kBrake);
        
        hingeMotor = new NEO(Constants.MANIPULATOR_HINGE_MOTOR, IdleMode.kBrake);
        hingePID = new PIDController(MANIPULATOR_KP, MANIPULATOR_KI, MANIPULATOR_KD);
    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator Encoder", getHingeAngle());
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

    
    public void setHingeUp(){
        hingePID.setSetpoint(MANIPULATOR_UP_ANGLE);
    }
    public void setHingeDown(){
        hingePID.setSetpoint(MANIPULATOR_DOWN_ANGLE);
    }
    public double getHingeAngle(){
        return hingeMotor.getPosition();
    }
    public void setHingePower(double power){
        double _power = Statics.clamp(power, MANIPULATOR_HINGE_MIN_POWER, MANIPULATOR_HINGE_MAX_POWER);
        
        if (Statics.withinError(getHingeAngle(), MANIPULATOR_DOWN_ANGLE, MANIPULATOR_POWER_OFF_ERROR) ||
            Statics.withinError(getHingeAngle(), MANIPULATOR_UP_ANGLE, MANIPULATOR_POWER_OFF_ERROR)){
            _power = 0.0;
            hingePID.reset();
        }
        hingeMotor.set(_power);
    }
    public void updateHingePID() {
        setHingePower( hingePID.calculate(getHingeAngle()) );
    }
    public void resetPID(){
        hingePID.reset();
    }
}
