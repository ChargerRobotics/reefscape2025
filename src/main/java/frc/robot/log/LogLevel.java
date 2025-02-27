package frc.robot.log;

public enum LogLevel {
    ERROR("ERROR"),
    WARNING("WARNING"),
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
            case ERROR, WARNING -> true;
            default -> false;
        };
    }

    @Override
    public String toString() {
        return name;
    }
}
