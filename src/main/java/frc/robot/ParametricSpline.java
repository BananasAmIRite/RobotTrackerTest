package frc.robot;

import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

public class ParametricSpline {

    private final List<TimedPath> paths;

    private ParametricSpline(List<TimedPath> paths) {
        this.paths = paths;
    }

    public double getTotalLength() {
        return getArcLengthAtTime(getTotalTime());
    }

    public double getArcLengthAtTime(double time) {
        return getArcLengthAtTime(time, 0.01);
    }

    public double getArcLengthAtTime(double time, double precision) {
        return Utils.integrate((t) -> Math.sqrt(Math.pow(getDxAtTime(t), 2) + Math.pow(getDyAtTime(t), 2)), 0, time, precision);
    }

    public double getTotalTime() {
        return this.paths.stream().reduce(0d, (partial, path) -> partial + path.getPath().getRunTime(), Double::sum);
    }

    private TimedPath getPathAtTime(double time) {
        double totalTime = 0;
        for (TimedPath p : paths) {
            totalTime += p.getPath().getRunTime();
            if (totalTime >= time) return p;
        }
        throw new RuntimeException(); // quite literally
    }

    public double getXAtTime(double time) {
        TimedPath path = getPathAtTime(time);
        return path.getPath().getXAt(time - path.getStartTime());
    }

    public double getYAtTime(double time) {
        TimedPath path = getPathAtTime(time);
        return path.getPath().getYAt(time - path.getStartTime());
    }

    public double getDxAtTime(double time) {
        TimedPath path = getPathAtTime(time);
        return path.getPath().getDxAt(time - path.getStartTime());
    }

    public double getDyAtTime(double time) {
        TimedPath path = getPathAtTime(time);
        return path.getPath().getDyAt(time - path.getStartTime());
    }

    public double getDdxAtTime(double time) {
        TimedPath path = getPathAtTime(time);
        return path.getPath().getDdxAt(time - path.getStartTime());
    }

    public double getDdyAtTime(double time) {
        TimedPath path = getPathAtTime(time);
        return path.getPath().getDdyAt(time - path.getStartTime());
    }

    public double signedCurvatureAt(double time) {
        return (getDxAtTime(time) * getDdyAtTime(time) + getDyAtTime(time) * getDdxAtTime(time)) / Math.pow(Math.pow(getDxAtTime(time), 2) + Math.pow(getDyAtTime(time), 2), 3.0/2);
    }

    public double signedRadiusAt(double time) {
        return 1/signedCurvatureAt(time);
    }

    public static ParametricSpline fromWaypoints(Waypoint[] waypoints) {
        return fromWaypoints(Arrays.stream(waypoints).collect(Collectors.toList()));
    }

    public static ParametricSpline fromWaypoints(List<Waypoint> waypoints) {
        List<TimedPath> paths = new ArrayList<>();
        double timeSoFar = 0;
        for (int i = 0; i < waypoints.size() - 1; i++) {
            Path p = new Path(waypoints.get(i), waypoints.get(i+1));
            paths.add(new TimedPath(p, timeSoFar));
            timeSoFar += p.getRunTime();
        }
        return new ParametricSpline(paths);
    }

    public static ParametricSpline fromFile(File file) throws IOException {
        JsonNode node = new ObjectMapper().readTree(file);
        JsonNode waypoints = node.get("waypoints");
        List<Waypoint> parsedWaypoints = new ArrayList<>();
        for (JsonNode waypoint : waypoints) {
            parsedWaypoints.add(new Waypoint(
                    waypoint.get("x").asDouble(),
                    waypoint.get("y").asDouble(),
                    waypoint.get("angle").asDouble(),
                    waypoint.get("weight").asDouble(),
                    waypoint.get("time").asDouble()
            ));
        }
        return fromWaypoints(parsedWaypoints);
    }

    public static void main(String[] args) throws IOException {
        TankMotionProfile prof = new TankMotionProfile(fromFile(new File("C:/Users/jason/Documents/cool.json")),
                new TankMotionProfile.TankMotionProfileConstraints(3, 3));
        double totalTime = prof.getTotalTime();
        for (double i = 0; i <= totalTime; i += 0.01) {
            System.out.println("current: " + prof.getStateAtTime(i));
            System.out.println("old: " + prof.getStateAtTime_old(i));
        }
    }
}
