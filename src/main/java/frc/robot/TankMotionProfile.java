package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class TankMotionProfile {

    private List<MotionProfileNode> nodes;
    private ParametricSpline spline;

    public TankMotionProfile(ParametricSpline spline, TankMotionProfileConstraints constraints) {
        this.spline = spline;
        this.nodes = calculateMotionProfile(spline, constraints, 0.5);
    }

    // TODO: gotta apply voltage constraints as well maybe
    private List<MotionProfileNode> calculateMotionProfile(ParametricSpline spline, TankMotionProfileConstraints constraints, double nodeLength) {
        List<MotionProfileNode> nodes = new ArrayList<>();

        {
            // NOTE: find the velocities first, then do acceleration and time
            // forward pass velocity
            MotionProfileNode lastNode = new MotionProfileNode(0, 0,
                    new Pose2d(
                            new Translation2d(
                                    spline.getXAtTime(0),
                                    spline.getYAtTime(0)
                            ),
                            new Rotation2d(Math.atan2(spline.getDyAtTime(0), spline.getDxAtTime(0)))
                    ),
                    spline.signedCurvatureAt(0), 0, 0, 0, 0
            );
            nodes.add(lastNode);

            for (double i = nodeLength; i < spline.getTotalLength(); i += nodeLength) {
                double splineTime = calculateTimeFromSplineDistance(spline, i, 0.1);
                double radius = spline.signedRadiusAt(splineTime);

                // vf^2 = v0^2+2ad
                double newLinearVelocity = Math.sqrt(Math.pow(lastNode.velocity, 2) + 2 * constraints.maxAcceleration * nodeLength);
                double angularVelocity = constraints.maxVelocity / (Math.abs(radius) + 1);
                double maxLinearVelocity = constraints.maxVelocity - angularVelocity;
                newLinearVelocity = Math.min(newLinearVelocity, maxLinearVelocity);

                MotionProfileNode node = new MotionProfileNode(newLinearVelocity, 0, new Pose2d(
                        new Translation2d(
                                spline.getXAtTime(splineTime),
                                spline.getYAtTime(splineTime)
                        ),
                        new Rotation2d(
                                Math.atan2(spline.getDyAtTime(splineTime), spline.getDxAtTime(splineTime))
                        )
                ), 1 / radius, 0, i, splineTime, 0);

                lastNode = node;
                nodes.add(node);
            }

        }

        // backward pass velocity
        {
            MotionProfileNode lastNode = nodes.get(nodes.size() - 1);
            lastNode.velocity = 0;
            lastNode.acceleration = 0;
            for (int i = nodes.size() - 2; i >= 0; i--) {
                MotionProfileNode curNode = nodes.get(i);
//                if (curNode.velocity - lastNode.velocity < 0) continue;
                double newLinearVelocity = Math.sqrt(Math.pow(lastNode.velocity, 2) + 2 * constraints.maxAcceleration * nodeLength);

                double radius = spline.signedRadiusAt(curNode.splineTime);
                double angularVelocity = constraints.maxVelocity / (Math.abs(radius) + 1);
                double maxLinearVelocity = constraints.maxVelocity - angularVelocity;
                newLinearVelocity = Math.min(curNode.velocity, Math.min(newLinearVelocity, maxLinearVelocity));

                curNode.velocity = newLinearVelocity;
                lastNode = curNode;
            }
        }

        double totalTime = 0;

        for (int i = 0; i < nodes.size() - 1; i++) {
            MotionProfileNode n = nodes.get(i);
            MotionProfileNode nextNode = nodes.get(i+1);

            double ds = nextNode.distanceTravelled - n.distanceTravelled;

            // a = (vf^2-v0^2)/2d
            n.acceleration = (Math.pow(nextNode.velocity, 2) - Math.pow(n.velocity, 2)) / (2 * ds);
            n.time = n.acceleration == 0 ? ds / n.velocity : (nextNode.velocity - n.velocity) / n.acceleration;

            nextNode.totalTime = totalTime + n.time;
            totalTime += n.time;
        }

        return nodes;
    }

    private double calculateTimeFromSplineDistance(ParametricSpline spline, double distance, double precision) {
        double timeLower = 0;
        double timeUpper = spline.getTotalTime();
        while (true) {
            double time = (timeLower + timeUpper) / 2;
            double dist = spline.getArcLengthAtTime(time);
            if (Math.abs(distance - dist) <= precision) return time;
            if (distance - dist < 0) timeUpper = time;
            if (distance - dist > 0) timeLower = time;
        }
    }

    public List<MotionProfileNode> getNodes() {
        return this.nodes;
    }

    public Trajectory.State getStateAtTime(double time) {
        double totalTime = 0;
        for (MotionProfileNode node : this.nodes) {
            if (totalTime <= time && totalTime + node.time >= time) {
                double dt = time - totalTime;
                double ds = node.velocity * dt + node.acceleration * dt * dt / 2;
                double splineTime = calculateTimeFromSplineDistance(spline, node.distanceTravelled + ds, 0.1); 

                return new Trajectory.State(
                        time,
                        node.velocity + node.acceleration * dt,
                        node.acceleration,
                        new Pose2d(
                                new Translation2d(spline.getXAtTime(splineTime), spline.getYAtTime(splineTime)),
                                new Rotation2d(Math.atan2(spline.getDyAtTime(splineTime), spline.getDxAtTime(splineTime)))
                        ),
                        node.curvature
                );
            }
            totalTime += node.time;
        }
        return null;
    }

    public double getTotalTime() {
        double totalTime = 0;
        for (MotionProfileNode node : this.nodes) {
            totalTime += node.time;
        }
        return totalTime;
    }

    public Trajectory asTrajectory() {
        return new Trajectory(this.getNodes().stream().map(MotionProfileNode::asState).collect(Collectors.toList()));
    }

    public static class MotionProfileNode {
        private double velocity;
        private double time;
        private Pose2d pose;
        private double curvature;
        private double acceleration;
        private double distanceTravelled;
        private double splineTime;
        private double totalTime;

        public MotionProfileNode(double velocity, double time, Pose2d pose, double curvature, double acceleration, double distanceTravelled, double splineTime, double totalTime) {
            this.velocity = velocity;
            this.time = time;
            this.pose = pose;
            this.curvature = curvature;
            this.acceleration = acceleration;
            this.distanceTravelled = distanceTravelled;
            this.splineTime = splineTime;
            this.totalTime = totalTime;
        }

        public double getVelocity() {
            return velocity;
        }

        public double getAcceleration() {
            return acceleration;
        }

        public double getTime() {
            return time;
        }

        public Trajectory.State asState() {
            return new Trajectory.State(totalTime, velocity, acceleration, pose, curvature);
        }

        @Override
        public String toString() {
            return "MotionProfileNode{" +
                    "velocity=" + velocity +
                    ", time=" + time +
                    ", pose=" + pose +
                    ", curvature=" + curvature +
                    ", acceleration=" + acceleration +
                    ", distanceTravelled=" + distanceTravelled +
                    ", splineTime=" + splineTime +
                    ", totalTime=" + totalTime +
                    '}';
        }
    }

    public static class TankMotionProfileConstraints {
        private final double maxVelocity;
        private final double maxAcceleration;

        public TankMotionProfileConstraints(double maxVel, double maxAccel) {
            this.maxVelocity = maxVel;
            this.maxAcceleration = maxAccel;
        }

        public double getMaxAcceleration() {
            return maxAcceleration;
        }

        public double getMaxVelocity() {
            return maxVelocity;
        }
    }
}
