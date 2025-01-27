package frc.robot.log;

import frc.robot.util.Elastic.Notification.NotificationLevel;

public enum LogLevel implements Comparable<LogLevel> {
    ERROR("ERROR"),
    WARN("WARN"),
    INFO("INFO"),
    DEBUG("DEBUG"),
    TRACE("TRACE"),
    ;

    private final String name;

    LogLevel(String name) {
        this.name = name;
    }

    public boolean isErr() {
        return switch (this) {
            case ERROR, WARN -> true;
            default -> false;
        };
    }

    public NotificationLevel toNotificationLevel() {
        return switch (this) {
            case ERROR -> NotificationLevel.ERROR;
            case WARN -> NotificationLevel.WARNING;
            case INFO -> NotificationLevel.INFO;
            case DEBUG, TRACE -> null;
        };
    }

    @Override
    public String toString() {
        return name;
    }
}
