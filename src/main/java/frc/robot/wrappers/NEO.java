package frc.robot.wrappers;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

public class NEO {
    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkMaxPIDController pidController;

    private int targetPosition = 0;
    private int targetVelocity = 0;

    public NEO(int id) {
        this(id, false);
    }

    public NEO(int id, CANSparkMax.IdleMode mode) {
        this(id, false, mode);
    }

    public NEO(int id, boolean reversed, CANSparkMax.IdleMode mode) {
        motor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);

        motor.setInverted(reversed);
        setIdleMode(mode);

        encoder = motor.getEncoder();
        pidController = motor.getPIDController();
    }

    public NEO(int id, boolean reversed) {
        this(id, reversed, CANSparkMax.IdleMode.kBrake);
    }

    public void setIdleMode(CANSparkMax.IdleMode mode) {
        motor.setIdleMode(mode);
    }

    public synchronized void set(double percent) {
        motor.set(percent);
    }

    public synchronized void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    public synchronized void setPosition(double position) {
        pidController.setReference(position, CANSparkMax.ControlType.kPosition);
        targetPosition = (int) position;
    }

    public synchronized void setPosition(double position, double arbitraryFeedForward) {
        pidController.setReference(position, CANSparkMax.ControlType.kPosition, 0, arbitraryFeedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
        targetPosition = (int) position;
    }

    public synchronized void setVelocity(double velocity) {
        pidController.setReference(velocity, CANSparkMax.ControlType.kVelocity);
        targetVelocity = (int) velocity;
    }

    public synchronized void setVelocity(double velocity, double arbitraryFeedForward) {
        pidController.setReference(velocity, CANSparkMax.ControlType.kVelocity, 0, arbitraryFeedForward, SparkMaxPIDController.ArbFFUnits.kVoltage);
        targetVelocity = (int) velocity;
    }

    public void follow(NEO host) {
        motor.follow(host.getMotor(), getMotor().getInverted());
    }

    public void follow(NEO host, boolean inverted) {
        motor.follow(host.getMotor(), inverted);
    }

    public double getPosition() {
        return encoder.getPosition();
    }

    public double getVelocity() {
        return encoder.getVelocity();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getTargetVelocity() {
        return targetVelocity;
    }

    public double getCurrent() { return motor.getOutputCurrent(); }

    public void setEncoder(double position) {
        encoder.setPosition(position);
    }

    public void burn() {
        motor.burnFlash();
    }

    // Set PID values
    public void configurePIDFF(double kP, double kI, double kD, double kIZ, double kFF) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
        pidController.setIZone(kIZ);
        pidController.setFF(kFF);
    }

    public void setOutputRange(double min, double max) {
        pidController.setOutputRange(min, max);
    }

    // get PID values
    public double getP() {
        return pidController.getP();
    }

    public double getI() {
        return pidController.getI();
    }

    public double getD() {
        return pidController.getD();
    }

    public double getIZ() {
        return pidController.getIZone();
    }

    public double getFF() {
        return pidController.getFF();
    }

    public int getID() {
        return motor.getDeviceId();
    }

    public CANSparkMax getMotor() {
        return motor;
    }

    public enum Control {
        DISABLED,
        VELOCITY,
        POSITION;
    }
}
