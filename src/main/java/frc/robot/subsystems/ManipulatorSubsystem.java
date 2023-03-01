package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.Constants;
import frc.robot.Constants.GamePieces;
import frc.robot.util.Statics;
import frc.robot.wrappers.NEO;

public class ManipulatorSubsystem extends SubsystemBase{
    private final NEO rollerMotor;

    private final double CONE_MOTOR_SPEED = -0.6;
    private final double CUBE_MOTOR_SPEED = 0.6;
    private final double DROP_MOTOR_SPEED = 0.75;
    private final double DROP_TIME = 1.5;

    private final NEO hingeMotor;
    private final PIDController hingePID;

    private final double MANIPULATOR_KP = 0.015;
    private final double MANIPULATOR_KI = 0.0001;
    private final double MANIPULATOR_KD = 0.00005;

    private final double MANIPULATOR_UP_ANGLE = 0;
    private final double MANIPULATOR_DOWN_ANGLE = 100;
    private final double MANIPULATOR_POWER_OFF_ERROR = 5;
    private final double MANIPULATOR_HINGE_MAX_POWER = 0.5;
    private final double MANIPULATOR_HINGE_MIN_POWER = -0.2;

    private final double MANIPULATOR_HINGE_GEAR_RATIO = 84.0/1.0;

    private GamePieces gamePiece = GamePieces.NONE;

    public ManipulatorSubsystem(){
        rollerMotor = new NEO(Constants.MANIPULATOR_ROLLER_MOTOR, false, IdleMode.kBrake);
        
        hingeMotor = new NEO(Constants.MANIPULATOR_HINGE_MOTOR, false, IdleMode.kBrake);
        hingePID = new PIDController(MANIPULATOR_KP, MANIPULATOR_KI, MANIPULATOR_KD);
        hingePID.setTolerance(MANIPULATOR_POWER_OFF_ERROR);
    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator Encoder", getHingeAngle());
    }

    public void intakeCone(){
        rollerMotor.set(CONE_MOTOR_SPEED);
        gamePiece = GamePieces.CONE;
    }
    public void dropCone(){
        rollerMotor.set(-DROP_MOTOR_SPEED);
        gamePiece = GamePieces.NONE;
    }

    public void intakeCube(){
        rollerMotor.set(CUBE_MOTOR_SPEED);
        gamePiece = GamePieces.CUBE;
    }
    public void dropCube(){
        rollerMotor.set(DROP_MOTOR_SPEED);
        gamePiece = GamePieces.NONE;
    }
    public GamePieces getGamePiece(){
        return gamePiece;
    }
    public void clearGamePiece(){
        gamePiece = GamePieces.NONE;
    }

    public void smartDrop(){
        if (gamePiece == GamePieces.CONE){
            dropCone();
        } else if (gamePiece == GamePieces.CUBE){
            dropCube();
        }
    }

    public void stop(){
        rollerMotor.set(0);
    }

    public void setHingeTarget(double angle){
        hingePID.setSetpoint(Statics.clamp(angle, MANIPULATOR_DOWN_ANGLE, MANIPULATOR_UP_ANGLE));
    }
    public void setHingeUp(){
        hingePID.setSetpoint(MANIPULATOR_UP_ANGLE);
    }
    public void setHingeDown(){
        hingePID.setSetpoint(MANIPULATOR_DOWN_ANGLE);
    }
    public double getHingeAngle(){
        return hingeMotor.getPosition()/MANIPULATOR_HINGE_GEAR_RATIO*360;
    }
    public void setHingePower(double power){
        double _power = Statics.clamp(power, MANIPULATOR_HINGE_MIN_POWER, MANIPULATOR_HINGE_MAX_POWER);
        SmartDashboard.putNumber("Hinge Power", _power);
        
        // if (Statics.withinError(getHingeAngle(), MANIPULATOR_DOWN_ANGLE, MANIPULATOR_POWER_OFF_ERROR) ||
        //     Statics.withinError(getHingeAngle(), MANIPULATOR_UP_ANGLE, MANIPULATOR_POWER_OFF_ERROR)){
        //     _power = 0.0;
        //     hingePID.reset();
        // }
        hingeMotor.set(_power);
    }
    public void updateHingePID() {
        setHingePower( hingePID.calculate(getHingeAngle()) );
    }
    public void resetPID(){
        hingePID.reset();
    }

    public boolean isHingeAtSetpoint(){
        return hingePID.atSetpoint();
    }
    public void resetEncoder(){
        hingeMotor.setEncoder(0);
    }


    /* Commands */
    public Command intakeConeCommand(){
        return Commands.startEnd(
            this::intakeCone,
            this::stop,
            this
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command intakeCubeCommand(){
        return Commands.startEnd(
            this::intakeCube,
            this::stop,
            this
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command dropConeCommand(){
        return Commands.sequence(
            new InstantCommand(this::dropCone, this),
            new WaitCommand(DROP_TIME),
            new InstantCommand(this::stop, this)
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command dropCubeCommand(){
        return Commands.sequence(
            new InstantCommand(this::dropCube, this),
            new WaitCommand(DROP_TIME),
            new InstantCommand(this::stop, this)
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command smartDropCommand(){
        return Commands.sequence(
            new InstantCommand(this::smartDrop, this),
            new WaitCommand(DROP_TIME),
            new InstantCommand(this::stop, this)
            );
    }

    public Command rawUpCommand(){
        return Commands.startEnd(() -> setHingePower(-MANIPULATOR_HINGE_MAX_POWER), () -> setHingePower(0), this);
    }
    public Command rawDownCommand(){
        return Commands.startEnd(() -> setHingePower(-MANIPULATOR_HINGE_MIN_POWER), () -> setHingePower(0), this);
    }

    public Command setHingeUpCommand(){
        return new InstantCommand(() -> setHingeUp());
    }
    public Command setHingeDownCommand(){
        return new InstantCommand(() -> setHingeDown());
    }
}
