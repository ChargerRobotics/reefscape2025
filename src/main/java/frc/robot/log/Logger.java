package frc.robot.log;

import java.util.HashMap;
import java.util.Map;
import java.util.function.BiConsumer;

public class Logger {
    private final Map<Class<?>, BiConsumer<?, Logger>> loggableMap = new HashMap<>();
    private final LoggerOptions options;

    public Logger(LoggerOptions options) {
        this.options = options;
    }

    public <T> void registerLoggable(Class<T> clazz, BiConsumer<T, Logger> loggable) {
        loggableMap.put(clazz, loggable);
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

    public <T> void logRegistered(T item, Class<T> itemClass) {
        //noinspection unchecked
        BiConsumer<T, Logger> loggable = (BiConsumer<T, Logger>) loggableMap.get(itemClass);
        if (loggable == null) throw new IllegalArgumentException("no mapping found for " + itemClass.getName());
        loggable.accept(item, this);
    }
}
