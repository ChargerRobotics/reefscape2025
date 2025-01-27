// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.datasiqn.robotutils.controlcurve.ControlCurve;
import com.datasiqn.robotutils.controlcurve.ControlCurves;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.pkl.ElevatorConfig;
import frc.robot.pkl.SwerveConfig;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.swerve.SwerveDrive;
import org.pkl.config.java.ConfigEvaluator;
import org.pkl.core.ModuleSource;

public class RobotContainer {
    public static final Logger LOGGER = new Logger(LoggerOptions.builder("Robot").build());

    private final DriveTrainSubsystem driveTrainSubsystem;
    private final ElevatorSubsystem elevatorSubsystem;

    private final XboxController controller = new XboxController(0);
    private final Pigeon2 pigeon = new Pigeon2(20);

    private final ControlCurve driveCurve = ControlCurves.power(3)
            .withDeadZone(0.1)
            .withPowerMultiplier(0.8)
            .build();
    private final ControlCurve rotateCurve = ControlCurves.power(3)
            .withDeadZone(0.1)
            .withPowerMultiplier(Math.PI)
            .build();

    public RobotContainer() {
        SwerveConfig swerveConfig;
        ElevatorConfig elevatorConfig;
        try (ConfigEvaluator evaluator = ConfigEvaluator.preconfigured()) {
            swerveConfig = evaluator.evaluate(ModuleSource.modulePath("swerve.pkl")).as(SwerveConfig.class);
            elevatorConfig = evaluator.evaluate(ModuleSource.modulePath("elevator.pkl")).as(ElevatorConfig.class);
        }

        this.driveTrainSubsystem = new DriveTrainSubsystem(new SwerveDrive(swerveConfig, pigeon::getRotation2d), () -> {
            ChassisSpeeds speeds = new ChassisSpeeds(
                    driveCurve.get(-controller.getLeftY()),
                    driveCurve.get(-controller.getLeftX()),
                    rotateCurve.get(-controller.getRightX())
            );
            // return ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pigeon.getRotation2d());
            return speeds;
        });
        this.elevatorSubsystem = new ElevatorSubsystem(elevatorConfig);
    }
}
