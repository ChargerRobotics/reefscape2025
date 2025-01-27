// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.datasiqn.robotutils.controlcurve.ControlCurve;
import com.datasiqn.robotutils.controlcurve.ControlCurves;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.log.LoggableREVLibError;
import frc.robot.log.Logger;
import frc.robot.log.LoggerOptions;
import frc.robot.pkl.ElevatorConfig;
import frc.robot.pkl.SwerveConfig;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.swerve.SwerveDrive;
import frc.robot.swerve.SwerveModule;
import frc.robot.util.Elastic;
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
            boolean fieldRelative = SmartDashboard.getBoolean("robot" + NetworkTable.PATH_SEPARATOR + "Field Relative", true);
            return fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(speeds, pigeon.getRotation2d()) : speeds;
        });
        this.elevatorSubsystem = new ElevatorSubsystem(elevatorConfig);

        SmartDashboard.putData("robot" + NetworkTable.PATH_SEPARATOR + "Swerve", driveTrainSubsystem.getDrive());
        SmartDashboard.putBoolean("robot" + NetworkTable.PATH_SEPARATOR + "Field Relative", true);

        SmartDashboard.putNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Rotate P", 0);
        SmartDashboard.putNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Rotate I", 0);
        SmartDashboard.putNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Rotate D", 0);
        SmartDashboard.putNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Max Acceleration", 0);
        SmartDashboard.putNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Max Velocity", 0);
        SmartDashboard.putData("swerve" + NetworkTable.PATH_SEPARATOR + "Update Rotate", Commands.runOnce(() -> {
            System.out.println("pressed");
            for (SwerveModule module : driveTrainSubsystem.getDrive().getModules()) {
                SparkMaxConfig config = new SparkMaxConfig();
                config.closedLoop
                        .p(SmartDashboard.getNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Rotate P", 0))
                        .i(SmartDashboard.getNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Rotate I", 0))
                        .d(SmartDashboard.getNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Rotate D", 0))
                        .maxMotion
                        .maxAcceleration(SmartDashboard.getNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Max Acceleration", 0))
                        .maxVelocity(SmartDashboard.getNumber("swerve" + NetworkTable.PATH_SEPARATOR + "Max Velocity", 0));
                SparkMax rotate = module.rotate();
                REVLibError error = rotate.configure(config, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
                LOGGER.log(new LoggableREVLibError("Error updating SparkMAX " + rotate.getDeviceId(), error));
            }
            Elastic.sendNotification(new Elastic.Notification(Elastic.Notification.NotificationLevel.INFO, "Updated SparkMAXes", "Updated SparkMAX configurations to Dashboard values"));
        }));
    }
}
