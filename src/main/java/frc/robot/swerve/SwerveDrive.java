package frc.robot.swerve;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.REVLibError;
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
import frc.robot.pkl.FRC;
import frc.robot.pkl.SwerveConfig;

import java.util.Arrays;
import java.util.function.Supplier;

public class SwerveDrive implements Sendable {
    private final SwerveDriveKinematics kinematics;
    private final SwerveModule[] modules;
    private final Supplier<Rotation2d> headingSupplier;

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
            Slot0Configs driveVelocityConfig = new Slot0Configs();
            driveVelocityConfig
                    .withKS(driveVelocityControl.kS)
                    .withKV(driveVelocityControl.kV)
                    .withKP(driveVelocityControl.kP)
                    .withKI(driveVelocityControl.kI)
                    .withKD(driveVelocityControl.kD);
            StatusCode configureDriveStatus = drive.getConfigurator().apply(driveVelocityConfig);
            if (!configureDriveStatus.isOK()) {
                String error = configureDriveStatus.getName() + " (" + configureDriveStatus.getDescription() + ")";
                if (configureDriveStatus.isWarning()) {
                    System.err.println("Warning when configuring swerve drive motor " + drive.getDeviceID() + ": " + error);
                } else if (configureDriveStatus.isError()) {
                    System.err.println("Error when configuring swerve drive motor " + drive.getDeviceID() + ": " + error);
                }
            }
            if (driveConfig.inverted) {
                // TODO: error messages on configure
                drive.getConfigurator().apply(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
            }

            FRC.PIDConstants rotatePositionControl = rotateConfig.positionControl;
            SparkBaseConfig rotatePositionConfig = new SparkMaxConfig();
            rotatePositionConfig.inverted(rotateConfig.inverted);
            rotatePositionConfig.encoder.positionConversionFactor(360 / rotateConfig.gearRatio);
            rotatePositionConfig.closedLoop
                    .maxOutput(0.7)
                    .minOutput(-0.7)
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
        }

        this.kinematics = new SwerveDriveKinematics(locations);
        this.states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
        this.headingSupplier = headingSupplier;
    }
    public SwerveDrive(Supplier<Rotation2d> headingSupplier, SwerveModule... modules) {
        this.kinematics = new SwerveDriveKinematics(Arrays.stream(modules).map(SwerveModule::location).toArray(Translation2d[]::new));
        this.modules = modules;
        this.headingSupplier = headingSupplier;
        this.states = kinematics.toSwerveModuleStates(new ChassisSpeeds());
    }

    public void setSpeeds(ChassisSpeeds speeds) {
        this.states = kinematics.toSwerveModuleStates(speeds);
    }

    public void update() {
        for (int i = 0; i < modules.length; i++) {
            SwerveModule module = modules[i];
            SwerveModuleState state = states[i];

            TalonFX drive = module.drive();
            SparkMax rotate = module.rotate();

            state.optimize(Rotation2d.fromDegrees(rotate.getEncoder().getPosition()));

            drive.set(state.speedMetersPerSecond);
            // double velocity = state.speedMetersPerSecond * module.driveGearRatio() / module.wheelCircumference();
            // StatusCode driveStatus = drive.setControl(new VelocityDutyCycle(velocity).withSlot(0));
            // if (!driveStatus.isOK()) {
            //     String error = driveStatus.getName() + " (" + driveStatus.getDescription() + ")";
            //     if (driveStatus.isWarning()) {
            //         System.err.println("Warning when setting swerve drive motor " + drive.getDeviceID() + ": " + error);
            //     } else if (driveStatus.isError()) {
            //         System.err.println("Error when setting swerve drive motor " + drive.getDeviceID() + ": " + error);
            //     }
            // }

            REVLibError setRotateResult = rotate.getClosedLoopController().setReference(state.angle.getDegrees(), SparkBase.ControlType.kMAXMotionPositionControl);
            RobotContainer.LOGGER.log(new LoggableREVLibError("Error setting SparkMAX " + rotate.getDeviceId(), setRotateResult));
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
