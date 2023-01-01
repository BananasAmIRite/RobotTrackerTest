package frc.robot;

public class TimedPath {
    private final Path path;
    private final double startTime;

    public TimedPath(Path p, double startTime) {
        this.path = p;
        this.startTime = startTime;
    }

    public Path getPath() {
        return path;
    }

    public double getStartTime() {
        return startTime;
    }

    public double getEndTime() {
        return startTime + this.path.getRunTime();
    }
}
