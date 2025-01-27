package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.pkl.ElevatorConfig;
import frc.robot.pkl.FRC;

import java.util.List;

public class ElevatorSubsystem extends SubsystemBase {
    private final SparkMax controller;
    private final ElevatorFeedforward feedforward;

    private final TrapezoidProfile profile;

    private TrapezoidProfile.State goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State setpoint = new TrapezoidProfile.State();

    public ElevatorSubsystem(ElevatorConfig config) {
        List<? extends FRC.MotorController> motors = config.motors;

        this.controller = new SparkMax(motors.get(0).id, SparkLowLevel.MotorType.kBrushless);
        for (int i = 1; i < motors.size(); i++) {
            FRC.MotorController controllerConfig = config.motors.get(i);
            SparkBaseConfig motorConfig = new SparkMaxConfig()
                    .follow(this.controller, controllerConfig.inverted);
            new SparkMax(controllerConfig.id, SparkLowLevel.MotorType.kBrushless)
                    .configure(motorConfig, SparkBase.ResetMode.kNoResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
        }

        ElevatorConfig.ElevatorFeedforwardConstants feedforwardConfig = config.feedforward;
        this.feedforward = new ElevatorFeedforward(feedforwardConfig.kS, feedforwardConfig.kG, feedforwardConfig.kV, feedforwardConfig.kA);

        FRC.Constraints constraintsConfig = config.constraints;
        this.profile = new TrapezoidProfile(new TrapezoidProfile.Constraints(constraintsConfig.maxVelocity, constraintsConfig.maxAcceleration));
    }
    public ElevatorSubsystem(SparkMax controller, ElevatorFeedforward feedforward, TrapezoidProfile profile) {
       this.controller = controller;
       this.feedforward = feedforward;
       this.profile = profile;
    }

    public void setGoal(TrapezoidProfile.State goal) {
        this.goal = goal;
    }
    public void setGoal(double goal) {
        this.goal.position = goal;
    }

    @Override
    public void periodic() {
        TrapezoidProfile.State nextState = profile.calculate(TimedRobot.kDefaultPeriod, setpoint, goal);

        controller.setVoltage(feedforward.calculateWithVelocities(setpoint.velocity, nextState.velocity));
        setpoint = nextState;
    }
}
