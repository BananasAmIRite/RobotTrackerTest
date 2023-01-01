package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

public class TankMotionProfile {
    private final List<MotionProfileNode> nodes;
    private ParametricSpline spline;

    public TankMotionProfile(ParametricSpline spline, TankMotionProfileConstraints constraints) {
        this.spline = spline;
        this.nodes = calculateMotionProfile(spline, constraints, 0.1);
    }

    public TankMotionProfile(List<MotionProfileNode> nodes) {
        this.nodes = nodes;
    }

    // TODO: offload most of the computations to preprocessing somewhere else
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
                double splineTime = calculateTimeFromSplineDistance(spline, i, 1E-1);
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
            double dist = spline.getArcLengthAtTime(time, precision);
            if (Math.abs(distance - dist) <= precision || Math.abs(timeUpper - timeLower) < 1E-9) return time; // either we've zeroed in on a time or the distance is precise enough
            if (distance - dist < 0) timeUpper = time;
            if (distance - dist > 0) timeLower = time;
        }
    }

    public List<MotionProfileNode> getNodes() {
        return this.nodes;
    }

    public Trajectory.State getStateAtTime_old(double time) {
        double totalTime = 0;
        for (MotionProfileNode node : this.nodes) {
            if (totalTime <= time && totalTime + node.time >= time) {
                double dt = time - totalTime;
                double ds = node.velocity * dt + node.acceleration * dt * dt / 2;
                double splineTime = calculateTimeFromSplineDistance(spline, node.distanceTravelled + ds, 1E-1);

                return new Trajectory.State(
                        time,
                        node.velocity + node.acceleration * dt,
                        node.acceleration,
                        new Pose2d(
                                new Translation2d(spline.getXAtTime(splineTime), spline.getYAtTime(splineTime)),
                                new Rotation2d(Math.atan2(spline.getDyAtTime(splineTime), spline.getDxAtTime(splineTime)))
                        ),
//                        node.curvature
                        spline.signedCurvatureAt(splineTime)
                );
            }
            totalTime += node.time;
        }
        return this.nodes.get(this.nodes.size() - 1).asState();
    }

    // less expensive to calculate, more preprocessing (small nodeLength) required for good results
    // simulates robot position using the average angular accelerations and velocities, which generates a curve close to the original curve
    // good thing is, this works without the original spline, so we can preprocess elsewhere and save to a file
    public Trajectory.State getStateAtTime(double time) {
        double totalTime = 0;  
        for (int i = 0; i < this.nodes.size()-1; i++) {
            MotionProfileNode node = this.nodes.get(i);
            MotionProfileNode nextNode = this.nodes.get(i+1);
            if (totalTime <= time && totalTime + node.time >= time) {
                double dt = time - totalTime;
                if (dt == 0) return node.asState();

                // https://www.desmos.com/calculator/zpzf9dpnh2

                double angularVelocity = node.velocity * node.curvature;
                double angularAcceleration = (nextNode.velocity * nextNode.curvature - node.velocity * node.curvature) / (node.time);

                double dx = Utils.integrate((t) -> (node.velocity + node.acceleration * t) *
                        Math.cos(node.pose.getRotation().getRadians() + angularVelocity * t + angularAcceleration * t * t / 2), 0, dt, dt*1E-2);
                double dy = Utils.integrate((t) -> (node.velocity + node.acceleration * t) *
                        Math.sin(node.pose.getRotation().getRadians() + angularVelocity * t + angularAcceleration * t * t / 2), 0, dt, dt*1E-2);

                double vx = (node.velocity + node.acceleration * dt) *
                        Math.cos(node.pose.getRotation().getRadians() + angularVelocity * dt + angularAcceleration * dt * dt / 2);
                double vy = (node.velocity + node.acceleration * dt) *
                        Math.sin(node.pose.getRotation().getRadians() + angularVelocity * dt + angularAcceleration * dt * dt / 2);

                double ax = node.acceleration * Math.cos(node.pose.getRotation().getRadians() + angularVelocity * dt + angularAcceleration * dt * dt / 2)
                        - (node.velocity + node.acceleration * dt)*(angularVelocity+angularAcceleration * dt) * Math.sin(node.pose.getRotation().getRadians() + angularVelocity * dt + angularAcceleration * dt * dt / 2);
                double ay = node.acceleration * Math.sin(node.pose.getRotation().getRadians() + angularVelocity * dt + angularAcceleration * dt * dt / 2)
                        + (node.velocity + node.acceleration * dt)*(angularVelocity+angularAcceleration * dt) * Math.cos(node.pose.getRotation().getRadians() + angularVelocity * dt + angularAcceleration * dt * dt / 2);

                return new Trajectory.State(
                        time,
                        node.velocity + node.acceleration * dt,
                        node.acceleration,
                        new Pose2d(
                                new Translation2d(node.pose.getX() + dx, node.pose.getY() + dy),
                                new Rotation2d(Math.atan2(vy, vx))
                        ),
                        (vx*ay-vy*ax)/Math.pow(vx*vx+vy*vy, 3/2.0)
                );
            }
            totalTime += node.time;
        }
        return this.nodes.get(this.nodes.size() - 1).asState();
    }

    public double getTotalTime() {
        return this.nodes.get(this.nodes.size() - 1).totalTime; 
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

        public Pose2d getPose() {
            return pose;
        }

        public double getCurvature() {
            return curvature;
        }

        public double getTotalTime() {
            return totalTime;
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
