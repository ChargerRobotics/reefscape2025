package frc.robot.log;

import com.ctre.phoenix6.StatusCode;

public class LoggableStatusCode implements Loggable {
    private final String title;
    private final StatusCode statusCode;

    public LoggableStatusCode(String title, StatusCode statusCode) {
        this.title = title;
        this.statusCode = statusCode;
    }

    @Override
    public void log(Logger logger) {
        if (statusCode.isOK()) return;

        String message = title + ": " + statusCode.getName() + " (" + statusCode.getDescription() + ")";
        if (statusCode.isWarning()) logger.log(message, LogLevel.WARN);
        else if (statusCode.isError()) logger.log(message, LogLevel.ERROR);
    }
}
