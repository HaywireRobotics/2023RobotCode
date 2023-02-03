package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

public class ArmSubsystem {
    private final NEO shoulderMotor;
    private final NEO elbowMotor;
    private final PIDController shoulderPID;
    private final PIDController elbowPID;
    private final DigitalInput shoulderLimit;
    private final DigitalInput elbowLimit;

    private final double SHOULDER_GEAR_REDUCTION = 1/9; // degrees of NEO to arm degrees
    private final double SHOULDER_MAX_ANGLE = 30;
    private final double SHOULDER_MIN_ANGLE = 0;
    private final double SHOULDER_MAX_RAISE_POWER = 1;
    private final double SHOULDER_MAX_LOWER_POWER = 0.5;

    private final double SHOULDER_KP = 0.04;
    private final double SHOULDER_KI = 0;
    private final double SHOULDER_KD = 0;
    private final double SHOULDER_AT_SETPOINT_POSITION_TOLERANCE = 1.5;
    private final double SHOULDER_AT_SETPOINT_VELOCITY_TOLERANCE = 0.5;
    private final double SHOULDER_MAX_WINDUP_UPPER = 1.0;
    private final double SHOULDER_MAX_WINDUP_LOWER = -0.1;

    private final double ELBOW_GEAR_REDUCTION = 1/9; // degrees of NEO to arm degrees
    private final double ELBOW_MAX_ANGLE = 30;
    private final double ELBOW_MIN_ANGLE = 0;
    private final double ELBOW_MAX_RAISE_POWER = 1;
    private final double ELBOW_MAX_LOWER_POWER = 0.5;

    private final double ELBOW_KP = 0.04;
    private final double ELBOW_KI = 0;
    private final double ELBOW_KD = 0;
    private final double ELBOW_AT_SETPOINT_POSITION_TOLERANCE = 1.5;
    private final double ELBOW_AT_SETPOINT_VELOCITY_TOLERANCE = 0.5;
    private final double ELBOW_MAX_WINDUP_UPPER = 1.0;
    private final double ELBOW_MAX_WINDUP_LOWER = -0.1;

    private double shoulderTarget;
    private double elbowTarget;

    private double shoulderHomingSpeed = -0.1;
    private double elbowHomingSpeed = -0.1;
    private boolean homing = false;
    private boolean shoulderHoming = false;
    private boolean elbowHoming = false;

    public ArmSubsystem(){
        shoulderMotor = new NEO(Constants.SHOULDER_MOTOR, IdleMode.kBrake);

        elbowMotor = new NEO(Constants.ELBOW_MOTOR, IdleMode.kBrake);
        
        shoulderPID = new PIDController(SHOULDER_KP, SHOULDER_KI, SHOULDER_KD);
        shoulderPID.setTolerance(SHOULDER_AT_SETPOINT_POSITION_TOLERANCE, SHOULDER_AT_SETPOINT_VELOCITY_TOLERANCE);
        shoulderPID.setIntegratorRange(SHOULDER_MAX_WINDUP_LOWER, SHOULDER_MAX_WINDUP_UPPER);

        shoulderLimit = new DigitalInput(Constants.SHOULDER_LIMIT);

        elbowPID = new PIDController(ELBOW_KP, ELBOW_KI, ELBOW_KD);
        elbowPID.setTolerance(ELBOW_AT_SETPOINT_POSITION_TOLERANCE, ELBOW_AT_SETPOINT_VELOCITY_TOLERANCE);
        elbowPID.setIntegratorRange(ELBOW_MAX_WINDUP_LOWER, ELBOW_MAX_WINDUP_UPPER);
        
        elbowLimit = new DigitalInput(Constants.ELBOW_LIMIT);
    }

    public void periodic() {
        if (elbowLimit.get()) {
            elbowMotor.setEncoder(ELBOW_MIN_ANGLE);
            elbowHoming = false;
        }

        if (shoulderLimit.get()) {
            shoulderMotor.setEncoder(SHOULDER_MIN_ANGLE);
            shoulderHoming = false;
        }

        if (shoulderHoming) {
            setShoulderPower(shoulderHomingSpeed);
        } else { 
            setShoulderPower(0); 
        }

        if (elbowHoming) {
            setElbowPower(elbowHomingSpeed);
        } else { 
            setElbowPower(0); 
        }

        homing = (elbowHoming || shoulderHoming);
       
        if (!homing) {
            updateShoulderPID();
            updateElbowPID();
        }
    }

    public void beginHoming(){
        homing = true;
    }

    private void setShoulderPower(double power){
        shoulderMotor.set(Math.min(Math.max(power, -SHOULDER_MAX_LOWER_POWER), SHOULDER_MAX_RAISE_POWER));
    }
    private void setElbowPower(double power){
        elbowMotor.set(Math.min(Math.max(power, -ELBOW_MAX_LOWER_POWER), ELBOW_MAX_RAISE_POWER));
    }

    public void setElbowTarget(double length){
        elbowTarget = Math.min(Math.max(length, ELBOW_MIN_ANGLE), ELBOW_MAX_ANGLE);
    }
    public void setShoulderTarget(double length){
        shoulderTarget = Math.min(Math.max(length, SHOULDER_MIN_ANGLE), SHOULDER_MAX_ANGLE);
    }

    public boolean shoulderReachedTarget(){
        return shoulderPID.atSetpoint();
    }
    public boolean elbowReachedTarget(){
        return elbowPID.atSetpoint();
    }
    public boolean reachedTarget(){
        return shoulderReachedTarget() && elbowReachedTarget();
    }

    public double getCurrentShoulder(){
        return shoulderMotor.getPosition()*SHOULDER_GEAR_REDUCTION;
    }
    public double getCurrentElbow(){
        return elbowMotor.getPosition()*ELBOW_GEAR_REDUCTION;
    }

    public void updateShoulderPID() {
        double output = shoulderPID.calculate(getCurrentShoulder(), shoulderTarget);
        setShoulderPower(output);
    }
    public void updateElbowPID() {
        double output = elbowPID.calculate(getCurrentElbow(), elbowTarget);
        setElbowPower(output);
    }
}
