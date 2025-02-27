package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPLTVController;

public class DriveTrainSubsystem extends SubsystemBase {
    private final SwerveDrive drive;

    private boolean fieldCentric = true;

    public DriveTrainSubsystem(SwerveDrive drive, Supplier<ChassisSpeeds> speedsSupplier, Supplier<Rotation2d> headingSupplier) {
        this.drive = drive;

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

    @Override
    public void periodic() {
        drive.update();
    }

    public SwerveDrive getDrive() {
        return drive;
    }
}
