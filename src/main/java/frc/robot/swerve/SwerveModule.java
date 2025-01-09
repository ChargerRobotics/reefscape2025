package frc.robot.swerve;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.geometry.Translation2d;

public record SwerveModule(TalonFX drive, SparkMax rotate, Translation2d location, double driveGearRatio, double wheelCircumference) {
}
