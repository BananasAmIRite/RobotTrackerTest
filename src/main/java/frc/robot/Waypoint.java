package frc.robot;

public class Waypoint {
    private final double runTime;
    private final double x;
    private final double y;
    private final double angle;
    private final double velocity;

    public Waypoint(double x, double y, double angle, double velocity, double runTime) {
        this.x = x;
        this.y = y;
        this.angle = angle;
        this.velocity = velocity;
        this.runTime = runTime;
    }

    public double getX() {
        return x;
    }

    public double getY() {
        return y;
    }

    public double getAngle() {
        return angle;
    }

    public double getVelocity() {
        return velocity;
    }

    public double getRunTime() {
        return runTime;
    }
}
