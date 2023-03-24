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

    private final double INTAKE_MOTOR_SPEED = -1.0;
    private final double SHOOT_SPEED = 1.0;
    private final double DROP_MOTOR_SPEED = 0.75;
    private final double DROP_TIME = 1.5;

    
    private GamePieces gamePiece = GamePieces.NONE;

    public RollerSubsystem(){
        rollerMotor = new NEO(Constants.MANIPULATOR_ROLLER_MOTOR, false, IdleMode.kBrake);
    }
    
    
    public void intake(){
        rollerMotor.set(INTAKE_MOTOR_SPEED);
        gamePiece = GamePieces.CONE;
    }
    public void drop(){
        rollerMotor.set(DROP_MOTOR_SPEED);
        gamePiece = GamePieces.NONE;
    }
    public void shoot() {
        rollerMotor.set(SHOOT_SPEED);
        gamePiece = GamePieces.NONE;
    }
    public GamePieces getGamePiece(){
        return gamePiece;
    }
    public void clearGamePiece(){
        gamePiece = GamePieces.NONE;
    }

    public void stop(){
        rollerMotor.set(0);
    }

    /*Commands */
    public Command intakeCommand(){
        return Commands.startEnd(
            this::intake,
            this::stop,
            this
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command startIntakeCommand() {
        return new InstantCommand(this::intake);
    }
    public Command dropCommand(){
        return Commands.startEnd(
            this::drop,
            this::stop,
            this
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command timedDropCommand(){
        return Commands.sequence(
            new InstantCommand(this::drop, this),
            new WaitCommand(DROP_TIME),
            new InstantCommand(this::stop, this)
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command shootCommand(){
        return Commands.startEnd(
            this::shoot,
            this::stop,
            this
            ).withInterruptBehavior(InterruptionBehavior.kCancelSelf);
    }
    public Command stopCommand(){
        return new InstantCommand(this::stop);
    }
}

