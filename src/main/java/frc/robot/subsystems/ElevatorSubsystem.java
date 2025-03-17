package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.ElevatorPositions;
import frc.robot.pkl.ElevatorConfig;
import frc.robot.pkl.FRC;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

import java.util.List;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax controller;
    private final PIDController pidController;
    private final ElevatorFeedforward feedforward;

    private final TrapezoidProfile profile;

    private final ElevatorPositions elevatorPositions;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public ElevatorSubsystem(ElevatorConfig config) {
        List<? extends FRC.MotorController> motors = config.motors;

        this.controller = new SparkMax(motors.get(0).id, SparkLowLevel.MotorType.kBrushless);
        SparkBaseConfig firstConfig = new SparkMaxConfig()
                .inverted(motors.get(0).inverted);
        this.controller.configure(firstConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        for (int i = 1; i < motors.size(); i++) {
            FRC.MotorController controllerConfig = config.motors.get(i);
            SparkBaseConfig motorConfig = new SparkMaxConfig()
                    .follow(this.controller, controllerConfig.inverted);
            new SparkMax(controllerConfig.id, SparkLowLevel.MotorType.kBrushless)
                    .configure(motorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        }

        FRC.PIDConstants pidConfig = config.pid;
        this.pidController = new PIDController(pidConfig.kP, pidConfig.kI, pidConfig.kD);

        ElevatorConfig.ElevatorFeedforwardConstants feedforwardConfig = config.feedforward;
        this.feedforward = new ElevatorFeedforward(feedforwardConfig.kS, feedforwardConfig.kG, feedforwardConfig.kV, feedforwardConfig.kA);

        ElevatorConfig.Constraints constraintsConfig = config.constraints;
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(constraintsConfig.maxVelocity, constraintsConfig.maxAcceleration));

        ElevatorConfig.ElevatorSetpoints setpoints = config.setpoints;
        this.elevatorPositions = new ElevatorPositions(setpoints.L1, setpoints.L2, setpoints.L3, setpoints.L4);

        SmartDashboard.putNumber("elevator/kP", this.pidController.getP());
        SmartDashboard.putNumber("elevator/kI", this.pidController.getI());
        SmartDashboard.putNumber("elevator/kD", this.pidController.getD());
        SmartDashboard.putNumber("elevator/kS", this.feedforward.getKs());
        SmartDashboard.putNumber("elevator/kG", this.feedforward.getKg());
        SmartDashboard.putNumber("elevator/kV", this.feedforward.getKv());
        SmartDashboard.putNumber("elevator/kA", this.feedforward.getKa());
    }
    public ElevatorSubsystem(SparkMax controller, PIDController pidController, ElevatorFeedforward feedforward, TrapezoidProfile profile, ElevatorPositions elevatorPositions) {
       this.controller = controller;
       this.pidController = pidController;
       this.feedforward = feedforward;
       this.profile = profile;
       this.elevatorPositions = elevatorPositions;
    }

    public SparkMax getController() {
        return controller;
    }

    public Command setZero() {
        return runOnce(() -> setGoal(0));
    }

    public Command setL1() {
        return runOnce(() -> setGoal(elevatorPositions.L1()));
    }

    public Command setL2() {
        return runOnce(() -> setGoal(elevatorPositions.L2()));
    }

    public Command setL3() {
        return runOnce(() -> setGoal(elevatorPositions.L3()));
    }

    public Command setL4() {
        return runOnce(() -> setGoal(elevatorPositions.L4()));
    }

    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }
    public void setGoal(double goal) {
        this.setGoal(new TrapezoidProfile.State(goal, 0));
    }

    public TrapezoidProfile.State getGoal() {
        return this.goal;
    }

    public void setSetpoint(TrapezoidProfile.State setpoint) {
        this.setpoint = setpoint;
    }
    public void setSetpoint(double setpoint) {
        this.setSetpoint(new TrapezoidProfile.State(setpoint, 0));
    }

    @Override
    public void periodic() {
        TrapezoidProfile.State nextState = profile.calculate(TimedRobot.kDefaultPeriod, setpoint, goal);

        controller.setVoltage(pidController.calculate(controller.getEncoder().getPosition(), nextState.position) + feedforward.calculateWithVelocities(setpoint.velocity, nextState.velocity));
        setpoint = nextState;

        SmartDashboard.putNumber("elevator/position", controller.getEncoder().getPosition());
        SmartDashboard.putNumber("elevator/setpoint", setpoint.position);

        pidController.setP(SmartDashboard.getNumber("elevator/kP", 0));
        pidController.setI(SmartDashboard.getNumber("elevator/kI", 0));
        pidController.setD(SmartDashboard.getNumber("elevator/kD", 0));

        feedforward.setKs(SmartDashboard.getNumber("elevator/kS", 0));
        feedforward.setKg(SmartDashboard.getNumber("elevator/kG", 0));
        feedforward.setKv(SmartDashboard.getNumber("elevator/kV", 0));
        feedforward.setKa(SmartDashboard.getNumber("elevator/kA", 0));
    }
}
