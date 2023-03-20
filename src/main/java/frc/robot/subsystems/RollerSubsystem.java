package frc.robot.subsystems;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.wrappers.NEO;
import frc.robot.Constants;
import frc.robot.Constants.GamePieces;

public class RollerSubsystem extends SubsystemBase{
    private final NEO rollerMotor;

    private final double CONE_MOTOR_SPEED = -1.0;
    private final double CUBE_MOTOR_SPEED = 1.0;
    private final double SHOOT_CUBE_SPEED = -1.0;
    private final double DROP_MOTOR_SPEED = 0.75;
    private final double DROP_TIME = 1.5;

    
    private GamePieces gamePiece = GamePieces.NONE;

    public RollerSubsystem(){
        rollerMotor = new NEO(Constants.MANIPULATOR_ROLLER_MOTOR, false, IdleMode.kBrake);
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
    public void shootCube() {
        rollerMotor.set(SHOOT_CUBE_SPEED);
        gamePiece = GamePieces.NONE;
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

    /*Commands */
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
    public Command shootCubeCommand(){
        return Commands.startEnd(
            this::shootCube,
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
}

