// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.datasiqn.robotutils.controlcurve.ControlCurve;
import com.datasiqn.robotutils.controlcurve.ControlCurves;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
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
import frc.robot.util.AprilTagDetection;
import frc.robot.util.Vector2d;

import java.util.OptionalDouble;

import org.pkl.config.java.ConfigEvaluator;
import org.pkl.core.ModuleSource;

public class RobotContainer {
    public static final Logger LOGGER = new Logger(LoggerOptions.builder("Robot").build());

    private final DriveTrainSubsystem driveTrainSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;
    private final SlideSubsystem slideSubsystem;

    private final CommandXboxController controller = new CommandXboxController(0);
    private final Pigeon2 pigeon = new Pigeon2(20);

    private final AprilTagDetection aprilTag = new AprilTagDetection();

    private final SendableChooser<AutoType> autoChooser = new SendableChooser<>();

    private final ControlCurve driveCurve = ControlCurves.power(3)
            .withDeadZone(0.1)
            .withPowerMultiplier(5)
            .build();

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
            double speedFactor = SmartDashboard.getBoolean("Slow Mode", false) ? 0.25 : 1;

            Vector2d controllerVector = new Vector2d(-controller.getLeftY(), -controller.getLeftX());
            double speed = driveCurve.get(Math.min(controllerVector.magnitude(), 1));
            controllerVector = controllerVector.normalized()
                .mult(speed * speedFactor);

            return new ChassisSpeeds(
                    controllerVector.x(),
                    controllerVector.y(),
                    driveCurve.get(-controller.getRightX()) * speedFactor * Math.PI / 2
            );
        }, new PIDController(0.03, 0, 0), pigeon::getRotation2d);
        this.elevatorSubsystem = new ElevatorSubsystem(elevatorConfig);
        this.elevatorSubsystem.getController().getEncoder().setPosition(0);
        this.slideSubsystem = new SlideSubsystem(new PWMSparkMax(9));

        Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
        pigeon.getConfigurator().apply(pigeonConfig);

        setControllerBindings();
        setDashboardValues();
    }

    /**
     * Sets bindings for the Xbox Controller
     * <p>
     * Left joystick - drive
     * <p>
     * Right joystick - rotate (horizontal only)
     * <p>
     * a - elevator 0 position
     * <p>
     * b - elevator L2 position
     * <p>
     * x - elevator L3 position
     * <p>
     * y - elevator L4 position
     * <p>
     * LT - toggle field centric
     * <p>
     * LB - toggle slow mode
     * <p>
     * RT - outtake
     * <p>
     * RB - auto outtake
     * <p>
     * Left stick - toggle field centric
     * <p>
     * D-pad left - auto AprilTag align
     */
    private void setControllerBindings() {
        controller.rightTrigger()
            .whileTrue(slideSubsystem.outtakeCommand(0.8));

        controller.rightBumper()
            .onTrue(slideSubsystem.outtakeCommand(0.75).withTimeout(0.09));

        controller.leftTrigger()
            .onTrue(Commands.runOnce(() -> {
                boolean fieldCentric = SmartDashboard.getBoolean("Field Centric", true);
                SmartDashboard.putBoolean("Field Centric", !fieldCentric);
            }));

        controller.leftBumper()
            .onTrue(Commands.runOnce(() -> {
                boolean slowMode = SmartDashboard.getBoolean("Slow Mode", false);
                SmartDashboard.putBoolean("Slow Mode", !slowMode);
            }));

        controller.a()
            .onTrue(elevatorSubsystem.setL1());

        controller.b()
            .onTrue(elevatorSubsystem.setL2());

        controller.x()
            .onTrue(elevatorSubsystem.setL3());

        controller.y()
            .onTrue(elevatorSubsystem.setL4());

        controller.leftStick()
            .onTrue(Commands.runOnce(() -> {
                boolean fieldCentric = SmartDashboard.getBoolean("Field Centric", true);
                SmartDashboard.putBoolean("Field Centric", !fieldCentric);
            }));

        controller.povUp()
            .whileTrue(Commands.run(() -> elevatorSubsystem.setGoal(elevatorSubsystem.getGoal().position + 0.2), elevatorSubsystem));

        controller.povDown()
            .whileTrue(Commands.run(() -> elevatorSubsystem.setGoal(elevatorSubsystem.getGoal().position - 0.2), elevatorSubsystem));

        controller.povLeft()
            .whileTrue(driveTrainSubsystem.followYCommand(() -> {
                if (aprilTag.getCurrentId() == -1) return OptionalDouble.empty();
                return OptionalDouble.of(aprilTag.getHorizontalOffset());
            }));
    }

    private void setDashboardValues() {
        for (AutoType autoType : AutoType.values()) {
            autoChooser.addOption(autoType.toString(), autoType);
        }

        SmartDashboard.putData("Swerve", this.driveTrainSubsystem.getDrive());
        SmartDashboard.putData("Auto", this.autoChooser);
        SmartDashboard.putData("Reset Gyro", Commands.runOnce(() -> pigeon.reset()));
        SmartDashboard.putData("Reset Wheel Encoders", Commands.runOnce(() -> {
            for (SwerveModule module : driveTrainSubsystem.getDrive().getModules()) {
                module.rotate().getEncoder().setPosition(0);
            }
        }));
        SmartDashboard.putData("Reset Elevator", Commands.runOnce(() -> {
            elevatorSubsystem.getController().getEncoder().setPosition(0);
            elevatorSubsystem.setSetpoint(0);
            elevatorSubsystem.setGoal(0);
        }));
        SmartDashboard.putData("reset velocity", Commands.runOnce(() -> {
            velocity = 0;
        }));
        SmartDashboard.putBoolean("Field Centric", true);
        SmartDashboard.putBoolean("Slow Mode", false);
        SmartDashboard.putData("Reverse Outtake", slideSubsystem.reverseOuttakeCommand(0.2));
    }

    public Command getAutoCommand() {
        AutoType autoType = autoChooser.getSelected();
        if (autoType == null) autoType = AutoType.NONE;
        return autoType.getAutoCommand(this);
    }

    public Rotation2d getHeading() {
        return pigeon.getRotation2d();
    }

    public DriveTrainSubsystem getDriveTrainSubsystem() {
        return driveTrainSubsystem;
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

        driveTrainSubsystem.setFieldCentric(SmartDashboard.getBoolean("Field Centric", true));
    }    
}
