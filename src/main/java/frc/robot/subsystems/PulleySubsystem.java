package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.wrappers.NEO;

import frc.robot.util.Statics;

public class PulleySubsystem extends SubsystemBase{
    private final NEO pulleyMotor;
    private final PIDController pulleyPID;

    private final double PULLEY_KP = 0.04;
    private final double PULLEY_KI = 0;
    private final double PULLEY_KD = 0;

    private final double PULLEY_GEAR_REDUCTION = 1 / 84;
    private final double PULLEY_MAX_EXTENSION_DEGREES = 940;
    private final double PULLEY_MIN_EXTENSION_DEGREES = 0.0;
    private final double PULLEY_DEGREES_TO_INCHES = 2.5*2*Math.PI/360;
    private final double PULLEY_LENGTH_COLLAPSED_INCHES = 31;

    /*
     * Min Length is about 31" <- update this
     * Max Length from hinge is 6'
     * Pully is 2.5" <- verify this!
     */

    public PulleySubsystem(){
        pulleyMotor = new NEO(Constants.PULLEY_MOTOR, IdleMode.kBrake);
        pulleyPID = new PIDController(PULLEY_KP, PULLEY_KI, PULLEY_KD);
    }

    public void periodic() {
        SmartDashboard.putNumber("Pulley Length", getPositionInches());
        SmartDashboard.putNumber("Pulley Encoder", getPositionDegrees());
    }

    public double getPositionDegrees(){
        return pulleyMotor.getPosition()*PULLEY_GEAR_REDUCTION;
    }
    public double getPositionInches(){
        return getPositionDegrees() * PULLEY_DEGREES_TO_INCHES;
    }
    public void setTargetDegrees(double angle){
        double _angle = Statics.clamp(angle, PULLEY_MIN_EXTENSION_DEGREES, PULLEY_MAX_EXTENSION_DEGREES);
        pulleyPID.setSetpoint(_angle);
    }
    public void setTargetInches(double length){
        setTargetDegrees(length / PULLEY_DEGREES_TO_INCHES);
    }
    public void setExtensionLengthFromJoint(double length){
        setTargetInches(length - PULLEY_LENGTH_COLLAPSED_INCHES);
    }
    public double getExtensionLengthFromJoint(){
        return getPositionInches() + PULLEY_LENGTH_COLLAPSED_INCHES;
    }

    public void setMotorPower(double power){
        double _power = power;
        double currentPosition = getPositionDegrees();
        if(currentPosition >= PULLEY_MAX_EXTENSION_DEGREES) _power = Math.min(_power, 0.0);
        if(currentPosition <= PULLEY_MIN_EXTENSION_DEGREES) _power = Math.max(_power, 0.0);

        pulleyMotor.set(_power);
    }

    public void updatePID(){
        setMotorPower(pulleyPID.calculate(getPositionDegrees()));
    }
    public void resetPID(){
        pulleyPID.reset();
    }
}
