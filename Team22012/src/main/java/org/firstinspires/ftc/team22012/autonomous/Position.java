package org.firstinspires.ftc.team22012.autonomous;

public abstract class Position {
    double x;
    double y;
    public Position(double x, double y) {
        this.x = x;
        this.y = y;
    }
    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }
}
