package frc.robot.log;

public class LoggerOptions {
    private final String name;
    private final LogLevel maxLevel;

    public LoggerOptions(Builder builder) {
        this.name = builder.name;
        this.maxLevel = builder.maxLevel;
    }

    public String getName() {
        return name;
    }

    public LogLevel getMaxLevel() {
        return maxLevel;
    }

    public static Builder builder(String name) {
        return new Builder(name);
    }

    public static class Builder {
        private final String name;

        private LogLevel maxLevel = LogLevel.INFO;

        public Builder(String name) {
            this.name = name;
        }

        public Builder withMaxLevel(LogLevel level) {
            this.maxLevel = level;
            return this;
        }

        public LoggerOptions build() {
            return new LoggerOptions(this);
        }
    }
}
