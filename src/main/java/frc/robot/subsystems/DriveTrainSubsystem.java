package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;

import java.util.OptionalDouble;
import java.util.function.Supplier;

public class DriveTrainSubsystem extends SubsystemBase {
    private final SwerveDrive drive;
    private final PIDController followPid;

    private boolean fieldCentric = true;

    public DriveTrainSubsystem(SwerveDrive drive, Supplier<ChassisSpeeds> speedsSupplier, PIDController followPid, Supplier<Rotation2d> headingSupplier) {
        this.drive = drive;
        this.followPid = followPid;

        setDefaultCommand(run(() -> {
            ChassisSpeeds speeds = speedsSupplier.get();
            if (fieldCentric) speeds = ChassisSpeeds.fromFieldRelativeSpeeds(speeds, headingSupplier.get());
            drive.setSpeeds(speeds);
        }));

        /*
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (IOException exception) {
            throw new RuntimeException(exception);
        } catch (ParseException exception) {
            System.err.println("Cannot parse PathPlanner config file!");
            exception.printStackTrace();
            return;
        }

        AutoBuilder.configure(
            () -> new Pose2d(),
            _pose2d -> {},
            () -> ChassisSpeeds.fromFieldRelativeSpeeds(speedsSupplier.get(), headingSupplier.get()),
            (speeds, feedforwards) -> drive.setSpeeds(speeds),
            new PPLTVController(0.02),
            config, 
            () -> {
                Optional<Alliance> alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) return alliance.get() == Alliance.Red;
                return false;
            },
            this
        );
        */
    }

    public boolean isFieldCentric() {
        return this.fieldCentric;
    }

    public void setFieldCentric(boolean fieldCentric) {
        this.fieldCentric = fieldCentric;
    }

    public Command followYCommand(Supplier<OptionalDouble> displacementSupplier) {
        return run(() -> {
            OptionalDouble displacement = displacementSupplier.get();
            if (displacement.isEmpty()) return;
            drive.setSpeeds(new ChassisSpeeds(0, followPid.calculate(displacement.getAsDouble()), 0));
        });
    }

    @Override
    public void periodic() {
        drive.update();
    }

    public SwerveDrive getDrive() {
        return drive;
    }
}
