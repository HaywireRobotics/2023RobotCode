package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Statics;
import frc.robot.wrappers.NEO;

public class ElevatorSybsystem extends SubsystemBase {
    private final NEO elevatorMotor;
    private final PIDController elevatorPID;

    // FIXME: all these values need to be recalculated
    private final double DEGREES_TO_INCHES = 1/360; // Inches of elevator movement per degree of motor rotation
    private final double MAX_EXTENSION = 21; // Inches the elevator can extend
    private final double MIN_EXTENSION = 0; // Length when fully collapsed
    private final double MAX_EXTEND_POWER = 1;
    private final double MAX_COLLAPSE_POWER = 0.5;

    // Update all these:
    private final double PIVOT_TO_TOP_SPROCKET = 24.0;
    private final double PIVOT_TO_CHAIN_ATTACHMENT = 24.0;
    private final double PIVOT_TO_TOP_ANGLE = 20.0;

    private final double EXTENSION_KP = 0.04;
    private final double EXTENSION_KI = 0;
    private final double EXTENSION_KD = 0;
    private final double AT_SETPOINT_POSITION_TOLERANCE = 1.5;
    private final double AT_SETPOINT_VELOCITY_TOLERANCE = 0.5;
    private final double MAX_WINDUP_UPPER = 1.0;
    private final double MAX_WINDUP_LOWER = -0.1;
    private double target;


    public ElevatorSybsystem(){
        elevatorMotor = new NEO(Constants.ELEVATOR_MOTOR, IdleMode.kBrake);
        
        elevatorPID = new PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD);
        elevatorPID.setTolerance(AT_SETPOINT_POSITION_TOLERANCE, AT_SETPOINT_VELOCITY_TOLERANCE);
        elevatorPID.setIntegratorRange(MAX_WINDUP_LOWER, MAX_WINDUP_UPPER);
    }

    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", getCurrentPosition());
        SmartDashboard.putNumber("Elevator Encoder", elevatorMotor.getPosition());
        SmartDashboard.putNumber("Elevator Arm Angle", elevatorMotor.getTargetPosition());
    }
    public void setMotorPower(double power){
        double _power = Statics.clamp(power, -MAX_COLLAPSE_POWER, MAX_EXTEND_POWER);
        double currentPosition = getCurrentPosition();
        if(currentPosition >= MAX_EXTENSION) _power = Math.min(_power, 0.0);
        if(currentPosition <= MIN_EXTENSION) _power = Math.max(_power, 0.0);
        elevatorMotor.set(_power);
    }

    public void setTarget(double length){
        target = Statics.clamp(length, MIN_EXTENSION, MAX_EXTENSION);
    }
    public void setTargetArmAngle(double angle){
        // Law of Cosines
        double theta = -angle + PIVOT_TO_TOP_ANGLE;
        double a = PIVOT_TO_TOP_SPROCKET;
        double b = PIVOT_TO_CHAIN_ATTACHMENT;
        double c = Math.sqrt(a*a+b*b-a*b*Math.cos(theta));
        double target = c;//Math.asin(angle)*(MAX_EXTENSION-MIN_EXTENSION) + MIN_EXTENSION; // PLZ UPDATE ME
        setTarget(target);
    }
    public double getArmAngle(){
        double a = PIVOT_TO_TOP_SPROCKET;
        double b = PIVOT_TO_CHAIN_ATTACHMENT;
        double c = getCurrentPosition();

        double theta = Math.acos((c*c-a*a-a*a)/(2*a*b));
        return -theta+PIVOT_TO_TOP_ANGLE;
    }

    public boolean reachedTarget(){
        return elevatorPID.atSetpoint();
    }
    public double getCurrentPosition(){
        return elevatorMotor.getPosition()*DEGREES_TO_INCHES;
    }

    public void updatePID(){
        double output = elevatorPID.calculate(getCurrentPosition(), target);
        setMotorPower(output);
    }
    public void resetPID(){
        elevatorPID.reset();
    }
}