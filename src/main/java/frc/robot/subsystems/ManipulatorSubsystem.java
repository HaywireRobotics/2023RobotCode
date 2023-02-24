package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.GamePieces;
import frc.robot.util.Statics;
import frc.robot.wrappers.NEO;

public class ManipulatorSubsystem extends SubsystemBase{
    private final NEO rollerMotor;

    private final double coneMotorSpeed = 0.1;
    private final double cubeMotorSpeed = -0.1;
    private final double dropMotorSpeed = 0.5;

    private final NEO hingeMotor;
    private final PIDController hingePID;

    private final double MANIPULATOR_KP = 0.04;
    private final double MANIPULATOR_KI = 0;
    private final double MANIPULATOR_KD = 0;

    private final double MANIPULATOR_UP_ANGLE = 90;
    private final double MANIPULATOR_DOWN_ANGLE = 0;
    private final double MANIPULATOR_POWER_OFF_ERROR = 5;
    private final double MANIPULATOR_HINGE_MAX_POWER = 0.5;
    private final double MANIPULATOR_HINGE_MIN_POWER = -0.2;

    private final double MANIPULATOR_HINGE_GEAR_RATIO = 1.0/1.0;

    private GamePieces gamePiece = GamePieces.NONE;

    public ManipulatorSubsystem(){
        rollerMotor = new NEO(Constants.MANIPULATOR_ROLLER_MOTOR, IdleMode.kBrake);
        
        hingeMotor = new NEO(Constants.MANIPULATOR_HINGE_MOTOR, IdleMode.kBrake);
        hingePID = new PIDController(MANIPULATOR_KP, MANIPULATOR_KI, MANIPULATOR_KD);
        hingePID.setTolerance(MANIPULATOR_POWER_OFF_ERROR);
    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator Encoder", getHingeAngle());
    }

    public void intakeCone(){
        rollerMotor.set(coneMotorSpeed);
        gamePiece = GamePieces.CONE;
    }
    public void dropCone(){
        rollerMotor.set(-dropMotorSpeed);
        gamePiece = GamePieces.NONE;
    }

    public void intakeCube(){
        rollerMotor.set(cubeMotorSpeed);
        gamePiece = GamePieces.CUBE;
    }
    public void dropCube(){
        rollerMotor.set(dropMotorSpeed);
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
        return hingeMotor.getPosition()*MANIPULATOR_HINGE_GEAR_RATIO;
    }
    public void setHingePower(double power){
        double _power = Statics.clamp(power, MANIPULATOR_HINGE_MIN_POWER, MANIPULATOR_HINGE_MAX_POWER);
        
        if (Statics.withinError(getHingeAngle(), MANIPULATOR_DOWN_ANGLE, MANIPULATOR_POWER_OFF_ERROR) ||
            Statics.withinError(getHingeAngle(), MANIPULATOR_UP_ANGLE, MANIPULATOR_POWER_OFF_ERROR)){
            _power = 0.0;
            hingePID.reset();
        }
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


    /* Commands */
    public Command intakeConeCommand(){
        return Commands.sequence(
            new InstantCommand(this::intakeCone, this),
            new WaitCommand(2),
            new InstantCommand(this::stop, this)
            );
    }
    public Command intakeCubeCommand(){
        return Commands.sequence(
            new InstantCommand(this::intakeCube, this),
            new WaitCommand(2),
            new InstantCommand(this::stop, this)
            );
    }
    public Command dropConeCommand(){
        return Commands.sequence(
            new InstantCommand(this::dropCone, this),
            new WaitCommand(2),
            new InstantCommand(this::stop, this)
            );
    }
    public Command dropCubeCommand(){
        return Commands.sequence(
            new InstantCommand(this::dropCube, this),
            new WaitCommand(2),
            new InstantCommand(this::stop, this)
            );
    }
    public Command smartDropCommand(){
        return Commands.sequence(
            new InstantCommand(this::smartDrop, this),
            new WaitCommand(2),
            new InstantCommand(this::stop, this)
            );
    }
}
