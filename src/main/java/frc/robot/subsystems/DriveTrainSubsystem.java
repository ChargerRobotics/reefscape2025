package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.swerve.SwerveDrive;

import java.util.function.Supplier;

public class DriveTrainSubsystem extends SubsystemBase {
    private final SwerveDrive drive;

    public DriveTrainSubsystem(SwerveDrive drive, Supplier<ChassisSpeeds> speedsSupplier) {
        this.drive = drive;

        setDefaultCommand(run(() -> drive.setSpeeds(speedsSupplier.get())));
    }

    @Override
    public void periodic() {
        drive.update();
    }

    public SwerveDrive getDrive() {
        return drive;
    }
}
