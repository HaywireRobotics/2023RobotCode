package frc.robot.wrappers;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.util.Vector;

public class SwerveModule {

    private double ROTATION_KP = 0.0048; // 0.003
    private double ROTATION_KI = 0.0025; //0.00  // 0.0015
    private double ROTATION_KD = 0.00001;
    private double ROTATION_KIZ = 0;
    private double ROTATION_KFF = 0;
    private double INTEGRATOR_RANGE = 0.01;
    private double DRIVE_KP = 0.00007; //6.5e-5;
    private double DRIVE_KI = 0.0;  //5.5e-7;
    private double DRIVE_KD = 0.0; //0.001;
    private double DRIVE_KIZ = 0;
    private double DRIVE_KFF = 0;

    private boolean enabled = false;

    private final double OFFSET;
    private double encoderOffset = 0;
    private double encoderScale = -360/12.8;

    private final PIDController rotationController = new PIDController(ROTATION_KP, ROTATION_KI, ROTATION_KD);
    private final PIDController driveController = new PIDController(DRIVE_KP, DRIVE_KI, DRIVE_KD);

    private SwerveModuleState desiredState;

    private final NEO rotationMotor;
    private final NEO driveMotor;

    private final CANCoder rotationEncoder;

    /* Odometry */
    private double rotationAngle;
    private double driveAngle;
    private double pRotationAngle;
    private double pDriveAngle;
    private Vector deltaPosition = new Vector(0., 0.);

    public SwerveModule(int driveID, int rotationID, int encoderID, double offset, boolean reverseDrive) {
        this(new NEO(driveID, reverseDrive), new NEO(rotationID), new CANCoder(encoderID), offset);
    }

    public SwerveModule(NEO driveMotor, NEO rotationMotor, CANCoder rotationEncoder, double offset) {
        this.rotationMotor = rotationMotor;
        this.driveMotor = driveMotor;
        this.rotationEncoder = rotationEncoder;

        rotationController.setIntegratorRange(-INTEGRATOR_RANGE, INTEGRATOR_RANGE);
        
        this.driveMotor.configurePIDFF(DRIVE_KP, DRIVE_KI, DRIVE_KD, DRIVE_KIZ, DRIVE_KFF);
        this.rotationMotor.configurePIDFF(ROTATION_KP, ROTATION_KI, ROTATION_KD, ROTATION_KIZ, ROTATION_KFF);

        this.rotationEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);

        this.OFFSET = offset;

        zeroEncoders();
    }

    public void setRotationPID(double kp, double ki, double kd) {
        ROTATION_KP = kp;
        ROTATION_KI = ki;
        ROTATION_KD = kd;
        rotationController.setPID(kp, ki, kd);
    }

    public void setDrivePID(double kp, double ki, double kd) {
        DRIVE_KP = kp;
        DRIVE_KI = ki;
        DRIVE_KD = kd;
        driveController.setPID(kp, ki, kd);
    }

    public void setState(SwerveModuleState state) {
        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(getRotationAbsolute()));

        desiredState = state;
        // System.out.println(state.speedMetersPerSecond);

        // driveMotor.setVelocity(state.speedMetersPerSecond / (WHEEL_DIAMETER * Math.PI));
        // rotationMotor.setPosition(state.angle.getDegrees());
        if( isEnabled() ){
            double driveCalc = driveController.calculate(this.getSpeedMetersPerSecond(), state.speedMetersPerSecond / (Constants.WHEEL_DIAMETER * Math.PI));

            // If drive is stoped, hold last angle.
            double stateAngle = Double.isNaN(state.angle.getDegrees()) ? pRotationAngle : state.angle.getDegrees();
            double rotationTarget = this.getRotation() + angleDifference(this.getRotation(), stateAngle);
                
            double rotateCalc = rotationController.calculate(this.getRotation(), rotationTarget);
            // if (this.printCount == 0) {
                // System.out.println(rotationMotor.getID() + "RotateCalc: " + rotateCalc + "\t" + this.getRotation() + "\t" + state.angle.getDegrees() + "\t" + encoderOffset);
            //     this.printCount = 1000;
            // };

            if (Math.abs(driveCalc) <= 0.01 && Math.abs(rotateCalc) <= 0.01) {
                zeroEncoders();
            }
            driveMotor.set(driveCalc*Math.cos(Math.toRadians(rotationController.getPositionError())));
            rotationMotor.set(-rotateCalc);
        }

        
        SmartDashboard.putNumber("SVAngle"+rotationEncoder.getDeviceID(), getRotation());
    }

    public void driveDirect(double driveSpeed, double rotationSpeed) {
        driveMotor.set(driveSpeed);
        rotationMotor.set(rotationSpeed);
    }

    public double getSpeedMetersPerSecond() {
        // return ((driveMotor.getVelocity() / 60) / 6.75) * (Constants.WHEEL_DIAMETER * Math.PI);
        return (driveMotor.getVelocity() / Constants.DRIVE_MOTOR_GEAR_RATIO) * (Constants.WHEEL_DIAMETER * Math.PI);
    }

    public double getRotationAbsolute() {
        return rotationEncoder.getAbsolutePosition() - OFFSET;
    }
    public double getRawRotationAbsolute() {
        return rotationEncoder.getAbsolutePosition();
    }
    public double getNeoRotation(){
        return this.rotationMotor.getPosition()*encoderScale;
    }
    public double getRotation() {
        double motorEncoderValue = getNeoRotation();
        return motorEncoderValue + encoderOffset;
    }

    public void zeroEncoders(){
        rotationMotor.setEncoder(0);
        encoderOffset = getRotationAbsolute();//-getNeoRotation();
        rotationController.reset();
    }

    public static double angleDifference( double angle1, double angle2 )
    {
        double diff = ( angle2 - angle1 + 180 ) % 360 - 180;
        return diff < -180 ? diff + 360 : diff;
    }

    // FIXME: idk what im doing
    public double getRotationMirrored() {
        double rotation = this.getRotation();
        if (rotation < 0) {
            rotation = 180 - rotation;
        }
        return rotation;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getSpeedMetersPerSecond(), Rotation2d.fromDegrees(getRotation()));
    }

    public SwerveModuleState getDesiredState() {
        return desiredState;
    }

    public int getID() {
        return rotationEncoder.getDeviceID();
    }

    public void putRawRotationSmartDashboard() {
        SmartDashboard.putNumber("CANCoder " + getID() + " Raw Rotation", getRawRotationAbsolute());
    }

    /* Odometry */
    public void updateOdometry(){
        this.pDriveAngle = this.driveAngle;
        this.pRotationAngle = this.rotationAngle;

        this.rotationAngle = this.getRotation();
        this.driveAngle = this.driveMotor.getPosition();

        double speed = (this.driveAngle - this.pDriveAngle);
        double direction = (this.pRotationAngle/2 + this.rotationAngle/2);

        this.deltaPosition.x = Math.sin(Math.toRadians(direction))*speed;
        this.deltaPosition.y =  Math.cos(Math.toRadians(direction))*speed;
    }

    public Vector getDeltaPosition(){
        return this.deltaPosition;
    }

    public Vector getVelocity(){
        return this.deltaPosition.scale(Constants.WHEEL_DIAMETER * Math.PI).scale(1/Constants.DRIVE_MOTOR_GEAR_RATIO);
    }

    public void disable(){
        enabled = false;
        rotationController.reset();
    }
    public void enable(){
        enabled = true;
    }
    public boolean isEnabled(){
        return enabled;
    }
}
