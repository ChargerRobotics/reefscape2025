package frc.robot.log;

public class Logger {
    private final LoggerOptions options;

    public Logger(LoggerOptions options) {
        this.options = options;
    }

    public void log(Loggable loggable) {
        loggable.log(this);
    }
    public void log(Object message, LogLevel level) {
        if (options.getMaxLevel().compareTo(level) < 0) return;

        String fullMessage = "[" + options.getName() + "] [" + level + "] " + message;
        if (level.isErr()) {
            System.err.println(fullMessage);
        } else {
            System.out.println(fullMessage);
        }
    }
}
