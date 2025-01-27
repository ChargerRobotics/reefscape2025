package frc.robot.log;

import com.revrobotics.REVLibError;

public class LoggableREVLibError implements Loggable {
    private final String title;
    private final REVLibError error;

    public LoggableREVLibError(String title, REVLibError error) {
        this.title = title;
        this.error = error;
    }

    @Override
    public void log(Logger logger) {
        if (error != REVLibError.kOk) logger.log(title + ": " + error.name(), LogLevel.ERROR);
    }
}
