package frc.robot.util;

public class Vector2d {
    private final double x;
    private final double y;

    public Vector2d() {
        this(0, 0);
    }
    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public Vector2d add(Vector2d vector2d) {
        return new Vector2d(x + vector2d.x, y + vector2d.y);
    }

    public Vector2d sub(Vector2d vector2d) {
        return new Vector2d(x - vector2d.x, y - vector2d.y);
    }

    public Vector2d mult(double scalar) {
        return new Vector2d(x * scalar, y * scalar);
    }

    public Vector2d div(double scalar) {
        return new Vector2d(x / scalar, y / scalar);
    }

    public Vector2d normalized() {
        double mag = magnitude();
        return div(mag);
    }

    public double magnitudeSquared() {
        return x * x + y * y;
    }

    public double magnitude() {
        return Math.sqrt(magnitudeSquared());
    }
}
