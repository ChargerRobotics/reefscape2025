package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.MotionMagicVelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.REVLibError;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.log.LoggableStatusCode;
import frc.robot.pkl.FRC;
import frc.robot.pkl.SwerveConfig;
import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

import java.util.Arrays;
import java.util.function.Supplier;

public class SwerveDrive implements Sendable {
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;
    private final Supplier<Rotation2d> headingSupplier;

    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();
    private SwerveModuleState[] states;

    public SwerveDrive(SwerveConfig config, Supplier<Rotation2d> headingSupplier) {
        int modulesSize = config.modules.size();

        this.modules = new SwerveModule[modulesSize];
        Translation2d[] locations = new Translation2d[modulesSize];

        for (int i = 0; i < modulesSize; i++) {
            SwerveConfig.SwerveModule module = config.modules.get(i);

            SwerveConfig.TalonFXMotor driveConfig = module.drive;
            SwerveConfig.SparkMAXMotor rotateConfig = module.rotate;

            TalonFX drive = new TalonFX(driveConfig.id);
            SparkMax rotate = new SparkMax(rotateConfig.id, SparkLowLevel.MotorType.kBrushless);

            SwerveConfig.TalonFXVelocityControl driveVelocityControl = driveConfig.velocityControl;
            Slot0Configs driveSlot0Config = new Slot0Configs();
            driveSlot0Config
                    .withKS(driveVelocityControl.kS)
                    .withKV(driveVelocityControl.kV)
                    .withKA(driveVelocityControl.kA)
                    .withKP(driveVelocityControl.kP)
                    .withKI(driveVelocityControl.kI)
                    .withKD(driveVelocityControl.kD);

            MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
            motionMagicConfigs
                .withMotionMagicAcceleration(driveVelocityControl.maxAcceleration * driveConfig.gearRatio / module.wheelCircumferenceMeters)
                .withMotionMagicCruiseVelocity(0);

            StatusCode configureDriveSlotStatus = drive.getConfigurator().apply(driveSlot0Config);
            RobotContainer.LOGGER.log(new LoggableStatusCode("Configuring drive slot 0 of talon " + drive.getDeviceID(), configureDriveSlotStatus));

            StatusCode configureDriveMMStatus = drive.getConfigurator().apply(motionMagicConfigs);
            RobotContainer.LOGGER.log(new LoggableStatusCode("Configuring motion magic of talon " + drive.getDeviceID(), configureDriveMMStatus));

            if (driveConfig.inverted) {
                StatusCode configureDriveInvertedStatus = drive.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
                RobotContainer.LOGGER.log(new LoggableStatusCode("Configuring inverted of talon " + drive.getDeviceID(), configureDriveInvertedStatus));
            }

            FRC.PIDConstants rotatePositionControl = rotateConfig.positionControl;
            SparkBaseConfig rotatePositionConfig = new SparkMaxConfig();
            rotatePositionConfig.inverted(rotateConfig.inverted);
            rotatePositionConfig.encoder.positionConversionFactor(360 / rotateConfig.gearRatio);
            rotatePositionConfig.closedLoop
                    .maxOutput(0.8)
                    .minOutput(-0.8)
                    .p(rotatePositionControl.kP)
                    .i(rotatePositionControl.kI)
                    .d(rotatePositionControl.kD)
                    .positionWrappingInputRange(-180, 180)
                    .positionWrappingEnabled(true);

            REVLibError configureRotateResult = rotate.configure(rotatePositionConfig, SparkBase.ResetMode.kResetSafeParameters, SparkBase.PersistMode.kNoPersistParameters);
            if (configureRotateResult != REVLibError.kOk) {
                System.err.println("Error when configuring swerve rotate motor " + rotate.getDeviceId() + ": " + configureRotateResult.name());
            }

            Translation2d location = new Translation2d(module.location.x, module.location.y);
            locations[i] = location;

            this.modules[i] = new SwerveModule(drive, rotate, location, driveConfig.gearRatio, module.wheelCircumferenceMeters);

            Elastic.sendNotification(new Notification(NotificationLevel.INFO, "Configured Swerve", ""));
        }

        this.kinematics = new SwerveDriveKinematics(locations);
        this.states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
        this.headingSupplier = headingSupplier;
    }
    public SwerveDrive(Supplier<Rotation2d> headingSupplier, SwerveModule... modules) {
        Translation2d[] locations = Arrays.stream(modules).map(SwerveModule::location).toArray(Translation2d[]::new);

        this.kinematics = new SwerveDriveKinematics(locations);
        this.modules = modules;
        this.headingSupplier = headingSupplier;
        this.states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        this.currentSpeeds = speeds;
        this.states = kinematics.toSwerveModuleStates(speeds);
    }

    public ChassisSpeeds getCurrentSpeeds() {
        return currentSpeeds;
    }

    public SwerveModule[] getModules() {
        return modules;
    }

    public void update() {
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];

            TalonFX drive = module.drive();
            SparkMax rotate = module.rotate();

            // if (state.speedMetersPerSecond == 0) state.angle = new Rotation2d();
            state.optimize(Rotation2d.fromDegrees(-rotate.getEncoder().getPosition()));

            // drive.set(state.speedMetersPerSecond);
            double velocity = state.speedMetersPerSecond * module.driveGearRatio() / module.wheelCircumference();
            MotionMagicVelocityDutyCycle velocityControl = new MotionMagicVelocityDutyCycle(velocity);
            SmartDashboard.putNumber("target acceleration", velocityControl.Acceleration);
            StatusCode driveStatus = drive.setControl(velocityControl.withSlot(0));
            if (!driveStatus.isOK()) {
                String error = driveStatus.getName() + " (" + driveStatus.getDescription() + ")";
                if (driveStatus.isWarning()) {
                    System.err.println("Warning when setting swerve drive motor " + drive.getDeviceID() + ": " + error);
                } else if (driveStatus.isError()) {
                    System.err.println("Error when setting swerve drive motor " + drive.getDeviceID() + ": " + error);
                }
            }

            REVLibError setRotateResult = rotate.getClosedLoopController().setReference(-state.angle.getDegrees(), SparkBase.ControlType.kPosition, ClosedLoopSlot.kSlot0, 0.19);
            if (setRotateResult != REVLibError.kOk) {
                System.err.println("Error when setting swerve rotate motor " + rotate.getDeviceId() + ": " + setRotateResult.name());
            }
        }
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> states[1].angle.getRadians(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> states[1].speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> states[2].angle.getRadians(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> states[2].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> states[3].angle.getRadians(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> states[3].speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> states[0].angle.getRadians(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> states[0].speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> headingSupplier.get().getRadians(), null);
    }
}
