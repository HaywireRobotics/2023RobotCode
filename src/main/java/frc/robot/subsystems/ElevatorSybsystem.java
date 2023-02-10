package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ElevatorSybsystem extends SubsystemBase {
    private final NEO elevatorMotor;
    private final PIDController elevatorPID;
    private final DigitalInput elevatorLimit;

    // FIXME: all these values need to be recalculated
    private final double DEGREES_TO_INCHES = 1/360; // Inches of elevator movement per degree of motor rotation
    private final double MAX_EXTENSION = 30; // Inches the elevator can extend
    private final double MIN_EXTENSION = 0; // Length when fully collapsed
    private final double MAX_EXTEND_POWER = 1;
    private final double MAX_COLLAPSE_POWER = 0.5;

    private final double EXTENSION_KP = 0.04;
    private final double EXTENSION_KI = 0;
    private final double EXTENSION_KD = 0;
    private final double AT_SETPOINT_POSITION_TOLERANCE = 1.5;
    private final double AT_SETPOINT_VELOCITY_TOLERANCE = 0.5;
    private final double MAX_WINDUP_UPPER = 1.0;
    private final double MAX_WINDUP_LOWER = -0.1;
    private double target;

    private double homingSpeed = -0.1;
    private boolean homing = false;

    public ElevatorSybsystem(){
        elevatorMotor = new NEO(Constants.ELEVATOR_MOTOR, IdleMode.kBrake);
        
        elevatorPID = new PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD);
        elevatorPID.setTolerance(AT_SETPOINT_POSITION_TOLERANCE, AT_SETPOINT_VELOCITY_TOLERANCE);
        elevatorPID.setIntegratorRange(MAX_WINDUP_LOWER, MAX_WINDUP_UPPER);

        elevatorLimit = new DigitalInput(Constants.ELEVATOR_EXTENSION_LIMIT);

    }

    public void periodic() {
        if(elevatorLimit.get()){
            elevatorMotor.setEncoder(MIN_EXTENSION);
            homing = false;
        }
        if(homing){
            setMotorPower(homingSpeed);
        }else{
            double output = elevatorPID.calculate(getCurrentExtension(), target);
            setMotorPower(output);
        }
    }
    public void home(){
        homing = true;
    }
    private void setMotorPower(double power){
        elevatorMotor.set(Math.min(Math.max(power, -MAX_COLLAPSE_POWER), MAX_EXTEND_POWER));
    }

    public void setExtensionTarget(double length){
        target = Math.min(Math.max(length, MIN_EXTENSION), MAX_EXTENSION);
    }
    public boolean reachedTarget(){
        return elevatorPID.atSetpoint();
    }
    public double getCurrentExtension(){
        return elevatorMotor.getPosition()*DEGREES_TO_INCHES;
    }
}