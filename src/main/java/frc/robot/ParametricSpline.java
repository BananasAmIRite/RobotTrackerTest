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

    private List<TimedPath> paths;

    private ParametricSpline(List<TimedPath> paths) {
        this.paths = paths;
    }

    public double getTotalLength() {
        return getArcLengthAtTime(getTotalTime());
    }

    public double getArcLengthAtTime(double time) {
        double arcLength = 0;
        double ARC_LENGTH_PRECISION = 0.01;
        for (double i = 0; i < time; i += ARC_LENGTH_PRECISION) {
            arcLength += Math.sqrt(Math.pow(getDxAtTime(i), 2) + Math.pow(getDyAtTime(i), 2)) * ARC_LENGTH_PRECISION;
        }
        return arcLength;
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
                    waypoint.get("velocity").asDouble(),
                    waypoint.get("time").asDouble()
            ));
        }
        return fromWaypoints(parsedWaypoints);
    }
}
