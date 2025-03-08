package frc.robot.util;

import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.IntegerSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;

public class AprilTagDetection {
    private final IntegerSubscriber idSubscriber = subscribeInt("tid");
    private final DoubleSubscriber horizontalOffsetSubscriber = subscribeDouble("tx");
    private final DoubleSubscriber verticalOffsetSubscriber = subscribeDouble("ty");

    public long getCurrentId() {
        return idSubscriber.get();
    }

    public double getHorizontalOffset() {
        return horizontalOffsetSubscriber.get();
    }

    public double getVerticalOffset() {
        return verticalOffsetSubscriber.get();
    }

    private static DoubleSubscriber subscribeDouble(String name) {
        return NetworkTableInstance.getDefault().getTable("limelight").getDoubleTopic(name).subscribe(-1);
    }

    private static IntegerSubscriber subscribeInt(String name) {
        return NetworkTableInstance.getDefault().getTable("limelight").getIntegerTopic(name).subscribe(-1);
    }
}
