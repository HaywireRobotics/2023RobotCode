package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DigitalSource;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.Statics;
import frc.robot.wrappers.NEO;

public class ElevatorSubsystem extends SubsystemBase {
    private final NEO elevatorMotor;
    private final PIDController elevatorPID;
    private final DigitalInput topLimitSwitch;

    private final DigitalInput elevatorEncoderInput;
    private final DutyCycleEncoder elevatorEncoder;

    // FIXME: all these values need to be recalculated
    private final double ELEVATOR_GEAR_RATIO = 36.0/1.0; // Gear ratio of the elevator
    private final double DEGREES_TO_INCHES = 2 * Math.PI; //1.5 // Inches of elevator movement per degree of motor rotation
    private final double MAX_HEIGHT = 26;//19.5; // Inches the elevator can extend
    private final double MIN_HEIGHT = 0; // Length when fully collapsed
    private final double MAX_RAISE_POWER = 1;
    private final double MAX_LOWER_POWER = 0.9;

    // Update all these:
    private final double PIVOT_TO_TOP_SPROCKET = 26;//23.75;
    private final double PIVOT_TO_CHAIN_ATTACHMENT = 23.5;
    private final double PIVOT_TO_TOP_ANGLE = 40.7;
    private final double CHAIN_WHEN_AT_TOP = 6;

    private final double EXTENSION_KP = 0.4;
    private final double EXTENSION_KI = 0.005;
    private final double EXTENSION_KD = 0.001;
    private final double AT_SETPOINT_POSITION_TOLERANCE = 1.5;
    private final double AT_SETPOINT_VELOCITY_TOLERANCE = 0.5;
    private final double MAX_WINDUP_UPPER = 1.0;
    private final double MAX_WINDUP_LOWER = -0.1;
    private double target;


    public ElevatorSubsystem(){
        elevatorMotor = new NEO(Constants.ELEVATOR_MOTOR, true, IdleMode.kBrake);
        topLimitSwitch = new DigitalInput(Constants.ELEVATOR_TOP_LIMIT_SWITCH);

        elevatorEncoderInput = new DigitalInput(Constants.ELEVATOR_ENCODER);
        elevatorEncoder = new DutyCycleEncoder(elevatorEncoderInput);
        
        elevatorPID = new PIDController(EXTENSION_KP, EXTENSION_KI, EXTENSION_KD);
        elevatorPID.setTolerance(AT_SETPOINT_POSITION_TOLERANCE, AT_SETPOINT_VELOCITY_TOLERANCE);
        elevatorPID.setIntegratorRange(MAX_WINDUP_LOWER, MAX_WINDUP_UPPER);
    }

    public void periodic() {
        SmartDashboard.putNumber("Elevator Height", getPosition());
        SmartDashboard.putNumber("Elevator Encoder", elevatorMotor.getPosition());
        SmartDashboard.putNumber("Elevator Arm Angle", elevatorMotor.getTargetPosition());
        SmartDashboard.putBoolean("Elevator Limit", getLimitSwitch());

        if(getLimitSwitch()){
            resetEncoder(MAX_HEIGHT);
        }
    }
    public void setMotorPower(double power){
        double _power = Statics.clamp(power, -MAX_LOWER_POWER, MAX_RAISE_POWER);
        if(getLimitSwitch()){
            _power = Math.min(_power, 0.0);
        }
        double currentPosition = getPosition();
        if(currentPosition >= MAX_HEIGHT && _power > 0){
            _power = 0;
            // resetPID();
        }
        if(currentPosition <= MIN_HEIGHT && _power < 0){
            _power = 0;
            // resetPID();
        }
        elevatorMotor.set(_power);
    }

    private boolean getLimitSwitch(){
        return topLimitSwitch.get();
    }

    public void setTarget(double length){
        target = Statics.clamp(length, MIN_HEIGHT, MAX_HEIGHT);
    }
    public void setTargetArmAngle(double angle){
        // Law of Cosines
        double theta = Math.toRadians(-angle + PIVOT_TO_TOP_ANGLE);
        double a = PIVOT_TO_TOP_SPROCKET;
        double b = PIVOT_TO_CHAIN_ATTACHMENT;
        double c = Math.sqrt(a*a+b*b-2*a*b*Math.cos(theta));
        double target = MAX_HEIGHT - (c-CHAIN_WHEN_AT_TOP);//Math.asin(angle)*(MAX_EXTENSION-MIN_EXTENSION) + MIN_EXTENSION; // PLZ UPDATE ME
        setTarget(target);
    }
    public double getArmAngle(){
        double a = PIVOT_TO_TOP_SPROCKET;
        double b = PIVOT_TO_CHAIN_ATTACHMENT;
        double c = MAX_HEIGHT - getPosition() + CHAIN_WHEN_AT_TOP;

        double theta = Math.toDegrees(Math.acos(-(c*c-a*a-b*b)/(2*a*b)));
        return -theta+PIVOT_TO_TOP_ANGLE;
    }
    private double getEncoder(){
        return elevatorEncoder.get();
    }
    public boolean isAtSetpoint(){
        return elevatorPID.atSetpoint();
    }
    public double getPosition(){
        return elevatorMotor.getPosition()/ELEVATOR_GEAR_RATIO*DEGREES_TO_INCHES;
    }
    public void resetEncoder(double inches){
        elevatorMotor.setEncoder(inches*ELEVATOR_GEAR_RATIO/DEGREES_TO_INCHES);
        elevatorEncoder.setPositionOffset(inches*ELEVATOR_GEAR_RATIO/DEGREES_TO_INCHES-(elevatorEncoder.get()-elevatorEncoder.getPositionOffset()));
    }

    public void updatePID(){
        double output = elevatorPID.calculate(getPosition(), target);
        setMotorPower(output);
    }
    public void resetPID(){
        elevatorPID.reset();
    }
    public void resetEncoder(){
        elevatorMotor.setEncoder(0);
    }

    /* Commands */
    public Command raiseArm(){
        return Commands.startEnd(() -> setMotorPower(MAX_RAISE_POWER), () -> setMotorPower(0), this);
    }
    public Command lowerArm(){
        return Commands.startEnd(() -> setMotorPower(-MAX_LOWER_POWER), () -> setMotorPower(0), this);
    }

}