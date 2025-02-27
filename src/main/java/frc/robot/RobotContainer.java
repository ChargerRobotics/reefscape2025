// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.datasiqn.robotutils.controlcurve.ControlCurve;
import com.datasiqn.robotutils.controlcurve.ControlCurves;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.log.Logger;
import frc.robot.log.LoggerOptions;
import frc.robot.pkl.ElevatorConfig;
import frc.robot.pkl.SwerveConfig;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SlideSubsystem;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveModule;
import frc.robot.util.Vector2d;

import org.pkl.config.java.ConfigEvaluator;
import org.pkl.core.ModuleSource;

public class RobotContainer {
    public static final Logger LOGGER = new Logger(LoggerOptions.builder("Robot").build());

    private final DriveTrainSubsystem driveTrainSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final SlideSubsystem slideSubsystem;

    private final CommandXboxController controller = new CommandXboxController(0);
    private final Pigeon2 pigeon = new Pigeon2(20);

    private final ControlCurve driveCurve = ControlCurves.power(3)
            .withDeadZone(0.1)
            .withPowerMultiplier(6)
            .build();

    private boolean slowMode = false;

    private double lastAcceleration = 0;
    private double velocity = 0;

    public RobotContainer() {
        SwerveConfig swerveConfig;
        ElevatorConfig elevatorConfig;
        try (ConfigEvaluator evaluator = ConfigEvaluator.preconfigured()) {
            swerveConfig = evaluator.evaluate(ModuleSource.modulePath("swerve.pkl")).as(SwerveConfig.class);
            elevatorConfig = evaluator.evaluate(ModuleSource.modulePath("elevator.pkl")).as(ElevatorConfig.class);
        }

        this.driveTrainSubsystem = new DriveTrainSubsystem(new SwerveDrive(swerveConfig, pigeon::getRotation2d), () -> {
            double speedFactor = slowMode ? 0.25 : 1;

            Vector2d controllerVector = new Vector2d(-controller.getLeftY(), -controller.getLeftX());
            double speed = driveCurve.get(Math.min(controllerVector.magnitude(), 1));
            controllerVector = controllerVector.normalized()
                .mult(speed * speedFactor);

            return new ChassisSpeeds(
                    controllerVector.x(),
                    controllerVector.y(),
                    driveCurve.get(-controller.getRightX()) * speedFactor * Math.PI / 2
            );
        }, pigeon::getRotation2d);
        this.elevatorSubsystem = new ElevatorSubsystem(elevatorConfig);
        this.slideSubsystem = new SlideSubsystem(new PWMSparkMax(1));

        controller.rightTrigger()
            .whileTrue(slideSubsystem.outtakeCommand());

        controller.leftBumper()
            .onTrue(Commands.runOnce(() -> slowMode = !slowMode));

        Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(pigeonConfig);

        SmartDashboard.putData("Swerve", this.driveTrainSubsystem.getDrive());
        SmartDashboard.putData("Reset Gyro", Commands.runOnce(() -> pigeon.reset()));
        SmartDashboard.putData("Reset Wheel Encoders", Commands.runOnce(() -> {
            for (SwerveModule module : driveTrainSubsystem.getDrive().getModules()) {
                module.rotate().getEncoder().setPosition(0);
            }
        }));
        SmartDashboard.putData("reset velocity", Commands.runOnce(() -> {
            velocity = 0;
        }));
        SmartDashboard.putBoolean("Field Centric", true);
    }

    public void periodic() {
        double acceleration = pigeon.getAccelerationY().getValue()
            .minus(LinearAcceleration.ofRelativeUnits(pigeon.getGravityVectorY().getValueAsDouble(), Units.Gs))
            .in(Units.MetersPerSecondPerSecond);
        velocity += (acceleration + lastAcceleration) / 2 * 0.02;
        lastAcceleration = acceleration;
        SmartDashboard.putNumber("acceleration", acceleration);
        SmartDashboard.putNumber("velocity", velocity);
        SmartDashboard.putNumber("should be velocity", driveTrainSubsystem.getDrive().getCurrentSpeeds().vxMetersPerSecond);

        SmartDashboard.putBoolean("Slow Mode", slowMode);

        driveTrainSubsystem.setFieldCentric(SmartDashboard.getBoolean("Field Centric", true));
    }
}
