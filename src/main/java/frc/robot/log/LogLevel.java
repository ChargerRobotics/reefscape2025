package frc.robot.log;

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

    @Override
    public String toString() {
        return name;
    }
}
