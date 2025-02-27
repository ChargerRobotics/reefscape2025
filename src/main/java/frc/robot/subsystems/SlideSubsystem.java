package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SlideSubsystem extends SubsystemBase {
    private final PWMSparkMax motorController;

    public SlideSubsystem(PWMSparkMax motorController) {
        this.motorController = motorController;
    }

    public Command outtakeCommand() {
        return startEnd(() -> motorController.set(-0.8), motorController::stopMotor);
    }
}
