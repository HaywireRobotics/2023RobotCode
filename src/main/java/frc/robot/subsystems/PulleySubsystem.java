package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private final double PULLEY_MAX_EXTENSION_DEGREES = 1.0;
    private final double PULLEY_MIN_EXTENSION_DEGREES = 0.0;
    private final double PULLEY_DEGREES_TO_METERS = 0.001;
    private final double PULLEY_LENGTH_COLLAPSED_METERS = 1.0;



    public PulleySubsystem(){
        pulleyMotor = new NEO(Constants.PULLEY_MOTOR, IdleMode.kBrake);
        pulleyPID = new PIDController(PULLEY_KP, PULLEY_KI, PULLEY_KD);
    }

    public void periodic() {
        
    }

    public double getPulleyPositionDegrees(){
        return pulleyMotor.getPosition()*PULLEY_GEAR_REDUCTION;
    }
    public double getPulleyPositionMeters(){
        return getPulleyPositionDegrees() * PULLEY_DEGREES_TO_METERS;
    }
    public void setPulleyTargetDegrees(double angle){
        double _angle = Statics.clamp(angle, PULLEY_MIN_EXTENSION_DEGREES, PULLEY_MAX_EXTENSION_DEGREES);
        pulleyPID.setSetpoint(_angle);
    }
    public void setPulleyTargetMeters(double length){
        setPulleyTargetDegrees(length / PULLEY_DEGREES_TO_METERS);
    }
    public void setExtensionLengthFromJoint(double length){
        setPulleyTargetMeters(length - PULLEY_LENGTH_COLLAPSED_METERS);
    }

    private void setPulleyPower(double power){
        double _power = power;
        double currentPosition = getPulleyPositionDegrees();
        if(currentPosition >= PULLEY_MAX_EXTENSION_DEGREES) _power = Math.min(_power, 0.0);
        if(currentPosition <= PULLEY_MIN_EXTENSION_DEGREES) _power = Math.max(_power, 0.0);

        pulleyMotor.set(_power);
    }
}
