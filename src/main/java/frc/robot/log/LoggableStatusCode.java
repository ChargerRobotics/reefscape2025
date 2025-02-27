package frc.robot.log;

import com.ctre.phoenix6.StatusCode;

import frc.robot.util.Elastic;
import frc.robot.util.Elastic.Notification;
import frc.robot.util.Elastic.Notification.NotificationLevel;

public class LoggableStatusCode implements Loggable {
    private final String action;
    private final StatusCode statusCode;

    public LoggableStatusCode(String action, StatusCode statusCode) {
        this.action = action;
        this.statusCode = statusCode;
    }

    @Override
    public void log(Logger logger) {
        if (statusCode.isOK()) return;

        LogLevel logLevel = LogLevel.INFO;
        NotificationLevel notificationLevel = NotificationLevel.INFO;
        if (statusCode.isWarning()) {
            logLevel = LogLevel.WARNING;
            notificationLevel = NotificationLevel.WARNING;
        }

        if (statusCode.isError()) {
            logLevel = LogLevel.ERROR;
            notificationLevel = NotificationLevel.ERROR;
        }

        logger.log(action + ": " + statusCode.getDescription(), logLevel);
        Elastic.sendNotification(new Notification(notificationLevel, action, statusCode.getDescription()));
    }
}
