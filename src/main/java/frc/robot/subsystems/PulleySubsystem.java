package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

import frc.robot.util.Statics;

public class PulleySubsystem extends SubsystemBase{
    private final NEO pulleyMotor;
    private final PIDController pulleyPID;
    private final DigitalInput pulleyLimitSwitch;

    private final double PULLEY_KP = 0.75;
    private final double PULLEY_KI = 0.01;
    private final double PULLEY_KD = 0.001;

    private final double PULLEY_GEAR_RATIO = 84.0 / 1.0 * 26/18;
    private final double PULLEY_ROTATIONS_TO_INCHES = 2.5*Math.PI;
    private final double PULLEY_MAX_EXTENSION_INCHES = 20.0; //PULLEY_DEGREES_TO_INCHES*55.5;
    private final double PULLEY_MIN_EXTENSION_INCHES = 0.0; //PULLEY_DEGREES_TO_INCHES*35.0;
    private final double PULLEY_LENGTH_COLLAPSED_INCHES = 31;

    private final double PULLEY_SETPOINT_POSITION_ERROR = 1.0;
    private final double PULLEY_SETPOINT_VELOCITY_ERROR = 1.0;

    /*
     * Min Length is about 35" <- update this
     * Max Length from hinge is 55.5"
     * Pully is 2.5" <- verify this!
     */

    public PulleySubsystem(){
        pulleyMotor = new NEO(Constants.PULLEY_MOTOR, true, IdleMode.kBrake);
        pulleyLimitSwitch = new DigitalInput(Constants.PULLEY_RETRACTED_LIMIT_SWITCH);

        pulleyPID = new PIDController(PULLEY_KP, PULLEY_KI, PULLEY_KD);
        pulleyPID.setTolerance(PULLEY_SETPOINT_POSITION_ERROR, PULLEY_SETPOINT_VELOCITY_ERROR);
    }

    public void periodic() {
        SmartDashboard.putNumber("Pulley Length", getPositionInches());
        SmartDashboard.putNumber("Pulley Encoder", pulleyMotor.getPosition()/PULLEY_GEAR_RATIO);
    }
    public double getPositionInches(){
        return pulleyMotor.getPosition()/PULLEY_GEAR_RATIO*PULLEY_ROTATIONS_TO_INCHES;
    }
    public void setTargetInches(double length){
        double _length = Statics.clamp(length, PULLEY_MIN_EXTENSION_INCHES, PULLEY_MAX_EXTENSION_INCHES);
        pulleyPID.setSetpoint(_length);
    }
    public void setExtensionLengthFromJoint(double length){
        setTargetInches(length - PULLEY_LENGTH_COLLAPSED_INCHES);
    }
    public double getExtensionLengthFromJoint(){
        return getPositionInches() + PULLEY_LENGTH_COLLAPSED_INCHES;
    }

    public void setMotorPower(double power){
        double _power = power;
        double currentPosition = getPositionInches();
        // if(getLimitSwitch()){
        //     if(_power < 0) _power = 0;
        //     resetEncoder(currentPosition);
        //     // resetPID();
        // }
        if(currentPosition >= PULLEY_MAX_EXTENSION_INCHES && _power >= 0) {
            _power = 0.0;
            // resetPID();
        }
        if(currentPosition <= PULLEY_MIN_EXTENSION_INCHES && _power <= 0){
            _power = 0.0;
            // resetPID();
        }

        pulleyMotor.set(_power);
    }
    private boolean getLimitSwitch(){
        // return !pulleyLimitSwitch.get();
        return false;
    }

    public void updatePID(){
        setMotorPower(pulleyPID.calculate(getPositionInches()));
    }
    public void resetPID(){
        pulleyPID.reset();
    }

    public boolean isAtSetpoint(){
        return pulleyPID.atSetpoint();
    }

    public void resetEncoder(double inches){
        pulleyMotor.setEncoder(inches/PULLEY_ROTATIONS_TO_INCHES*PULLEY_GEAR_RATIO);
    }

    public void resetEncoder(){
        pulleyMotor.setEncoder(0);
    }

    /* Commands */
    public Command extendCommand(){
        return Commands.startEnd(() -> setMotorPower(1.0), () -> setMotorPower(0), this);
    }
    public Command retractCommand(){
        return Commands.startEnd(() -> setMotorPower(-1.0), () -> setMotorPower(0), this);
    }
}
