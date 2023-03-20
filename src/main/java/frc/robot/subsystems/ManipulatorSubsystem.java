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


    private final NEO hingeMotor;
    private final PIDController hingePID;
    private final RollerSubsystem rollerSubsystem;

    private final double MANIPULATOR_KP = 0.012;
    private final double MANIPULATOR_KI = 0.0001;
    private final double MANIPULATOR_KD = 0.0006;

    private final double MANIPULATOR_UP_ANGLE = 0;
    private final double MANIPULATOR_DOWN_ANGLE = 220;
    private final double MANIPULATOR_POWER_OFF_ERROR = 5;
    private final double MANIPULATOR_HINGE_MAX_POWER = 0.85;
    private final double MANIPULATOR_HINGE_MIN_POWER = -0.25;

    private final double MANIPULATOR_HINGE_GEAR_RATIO = 84.0/1.0;


    public ManipulatorSubsystem(){
        
        hingeMotor = new NEO(Constants.MANIPULATOR_HINGE_MOTOR, false, IdleMode.kBrake);
        hingePID = new PIDController(MANIPULATOR_KP, MANIPULATOR_KI, MANIPULATOR_KD);
        hingePID.setTolerance(MANIPULATOR_POWER_OFF_ERROR);

        rollerSubsystem = new RollerSubsystem();

    }

    public void periodic() {
        SmartDashboard.putNumber("Manipulator Encoder", getHingeAngle());
    }

    public void setHingeTarget(double angle){
        hingePID.setSetpoint(Statics.clamp(angle, MANIPULATOR_UP_ANGLE, MANIPULATOR_DOWN_ANGLE));
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
    public void intakeCone(){
        rollerSubsystem.intakeCone();
    }
    public void dropCone(){
        rollerSubsystem.dropCone();
    }

    public void intakeCube(){
        rollerSubsystem.intakeCube();
    }
    public void shootCube() {
        rollerSubsystem.shootCube();
    }

    public void dropCube(){
        rollerSubsystem.dropCube();
    }
    
    public GamePieces getGamePiece(){
        return rollerSubsystem.getGamePiece();
    }
    public void stop(){
        rollerSubsystem.stop();
    }


    /* Commands */
    public Command intakeConeCommand(){
        return rollerSubsystem.intakeConeCommand();
    }
    public Command intakeCubeCommand(){
        return rollerSubsystem.intakeCubeCommand();
    }
    public Command shootCubeCommand(){
        return rollerSubsystem.shootCubeCommand();
    }
    public Command dropConeCommand(){
        return rollerSubsystem.dropConeCommand();
    }
    public Command dropCubeCommand(){
        return rollerSubsystem.dropCubeCommand();
    }
    public Command smartDropCommand(){
        return rollerSubsystem.smartDropCommand();
    }

    public Command rawUpCommand(){
        return Commands.startEnd(this::rawUp, () -> setHingePower(0), this);
    }
    public void rawUp() {
        setHingePower(-MANIPULATOR_HINGE_MAX_POWER);
    }
    public Command rawDownCommand(){
        return Commands.startEnd(this::rawDown, () -> setHingePower(0), this);
    }
    public void rawDown() {
        setHingePower(-MANIPULATOR_HINGE_MIN_POWER);
    }

    public Command setHingeTargetCommand(double angle) {
        return new InstantCommand(() -> setHingeTarget(angle));
    }
    public Command setHingeUpCommand(){
        return new InstantCommand(() -> setHingeUp());
    }
    public Command setHingeDownCommand(){
        return new InstantCommand(() -> setHingeDown());
    }
}
