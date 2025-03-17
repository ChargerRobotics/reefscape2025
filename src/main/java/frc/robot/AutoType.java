package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public enum AutoType {
    NONE,
    BACKWARDS_1_SEC,
    BACKWARDS_05_SEC,
    ;

    public Command getAutoCommand(RobotContainer robotContainer) {
        return switch (this) {
            case NONE -> Commands.none();
            case BACKWARDS_1_SEC -> Commands.startEnd(() -> {
                robotContainer.getDriveTrainSubsystem().getDrive().setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(-3, 0, 0, robotContainer.getHeading()));
            }, () -> {
                robotContainer.getDriveTrainSubsystem().getDrive().setSpeeds(new ChassisSpeeds());
            }, robotContainer.getDriveTrainSubsystem())
                .withTimeout(1);
            case BACKWARDS_05_SEC -> Commands.startEnd(() -> {
                robotContainer.getDriveTrainSubsystem().getDrive().setSpeeds(ChassisSpeeds.fromFieldRelativeSpeeds(-3, 0, 0, robotContainer.getHeading()));
            }, () -> {
                robotContainer.getDriveTrainSubsystem().getDrive().setSpeeds(new ChassisSpeeds());
            }, robotContainer.getDriveTrainSubsystem())
                .withTimeout(0.5);
        };
    }

    @Override
    public String toString() {
        return switch (this) {
            case NONE -> "None";
            case BACKWARDS_1_SEC -> "Move backwards (1 sec)";
            case BACKWARDS_05_SEC -> "Move backwards (0.5 sec)";
        };
    }
}
