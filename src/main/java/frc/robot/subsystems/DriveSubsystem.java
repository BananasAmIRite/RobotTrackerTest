// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

// drivetrain subsystem for testing
public class DriveSubsystem extends SubsystemBase {
    // The motors on the left side of the drive.
    private final MotorControllerGroup leftMotors =
            new MotorControllerGroup(
                    new PWMSparkMax(DriveConstants.kLeftMotor1Port),
                    new PWMSparkMax(DriveConstants.kLeftMotor2Port));

    // The motors on the right side of the drive.
    private final MotorControllerGroup rightMotors =
            new MotorControllerGroup(
                    new PWMSparkMax(DriveConstants.kRightMotor1Port),
                    new PWMSparkMax(DriveConstants.kRightMotor2Port));

    // The robot's drive
    private final DifferentialDrive drive = new DifferentialDrive(leftMotors, rightMotors);

    // The left-side drive encoder
    private final Encoder leftEncoder =
            new Encoder(
                    DriveConstants.kLeftEncoderPorts[0],
                    DriveConstants.kLeftEncoderPorts[1],
                    DriveConstants.kLeftEncoderReversed);

    // The right-side drive encoder
    private final Encoder rightEncoder =
            new Encoder(
                    DriveConstants.kRightEncoderPorts[0],
                    DriveConstants.kRightEncoderPorts[1],
                    DriveConstants.kRightEncoderReversed);

    // The gyro sensor
    private final ADXRS450_Gyro gyro = new ADXRS450_Gyro();

    // Odometry class for tracking robot pose
    private final DifferentialDriveOdometry odometry;

    // These classes help us simulate our drivetrain
    public DifferentialDrivetrainSim drivetrainSimulator;
    private final EncoderSim leftEncoderSim;
    private final EncoderSim rightEncoderSim;
    // The Field2d class shows the field in the sim GUI
    private final Field2d fieldSim;
    private final ADXRS450_GyroSim gyroSim;

    private final DifferentialDriveKinematics kinematics;

    private final NetworkTableEntry xEntry;
    private final NetworkTableEntry yEntry;

    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        // We need to invert one side of the drivetrain so that positive voltages
        // result in both sides moving forward. Depending on how your robot's
        // gearbox is constructed, you might have to invert the left side instead.
        rightMotors.setInverted(true);

        // Sets the distance per pulse for the encoders
        leftEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);
        rightEncoder.setDistancePerPulse(DriveConstants.kEncoderDistancePerPulse);

        resetEncoders();
        odometry =
                new DifferentialDriveOdometry(
                        gyro.getRotation2d());

        if (RobotBase.isSimulation()) { // If our robot is simulated
            // This class simulates our drivetrain's motion around the field.
            drivetrainSimulator =
                    new DifferentialDrivetrainSim(
                            DriveConstants.kDrivetrainPlant,
                            DriveConstants.kDriveGearbox,
                            DriveConstants.kDriveGearing,
                            DriveConstants.kTrackwidthMeters,
                            DriveConstants.kWheelDiameterMeters / 2.0,
                            VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

            // The encoder and gyro angle sims let us set simulated sensor readings
            leftEncoderSim = new EncoderSim(leftEncoder);
            rightEncoderSim = new EncoderSim(rightEncoder);
            gyroSim = new ADXRS450_GyroSim(gyro);

            // the Field2d class lets us visualize our robot in the simulation GUI.
            fieldSim = new Field2d();
            SmartDashboard.putData("Field", fieldSim);
        } else {
            leftEncoderSim = null;
            rightEncoderSim = null;
            gyroSim = null;

            fieldSim = null;
        }

        kinematics = new DifferentialDriveKinematics(DriveConstants.kTrackwidthMeters);
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        xEntry = inst.getEntry("XPos");
        yEntry = inst.getEntry("YPos");
    }

    @Override
    public void periodic() {
        // Update the odometry in the periodic block
        odometry.update(
                Rotation2d.fromDegrees(getHeading()),
                leftEncoder.getDistance(),
                rightEncoder.getDistance());

        var translation = odometry.getPoseMeters().getTranslation();
        xEntry.setNumber(translation.getX());
        yEntry.setNumber(translation.getY());

    }

    @Override
    public void simulationPeriodic() {
        // To update our simulation, we set motor voltage inputs, update the simulation,
        // and write the simulated positions and velocities to our simulated encoder and gyro.
        // We negate the right side so that positive voltages make the right side
        // move forward.
        drivetrainSimulator.setInputs(
                leftMotors.get() * RobotController.getBatteryVoltage(),
                rightMotors.get() * RobotController.getBatteryVoltage());
        drivetrainSimulator.update(0.020);

        leftEncoderSim.setDistance(drivetrainSimulator.getLeftPositionMeters());
        leftEncoderSim.setRate(drivetrainSimulator.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(drivetrainSimulator.getRightPositionMeters());
        rightEncoderSim.setRate(drivetrainSimulator.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-drivetrainSimulator.getHeading().getDegrees());
        fieldSim.setRobotPose(getPose());
        SmartDashboard.putData("field", fieldSim);
    }

    /**
     * Returns the current being drawn by the drivetrain. This works in SIMULATION ONLY! If you want
     * it to work elsewhere, use the code in {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
     *
     * @return The drawn current in Amps.
     */
    public double getDrawnCurrentAmps() {
        return drivetrainSimulator.getCurrentDrawAmps();
    }

    /**
     * Returns the currently-estimated pose of the robot.
     *
     * @return The pose.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Returns the current wheel speeds of the robot.
     *
     * @return The current wheel speeds.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    /**
     * Resets the odometry to the specified pose.
     *
     * @param pose The pose to which to set the odometry.
     */
    public void resetOdometry(Pose2d pose) {
        resetEncoders();
        drivetrainSimulator.setPose(pose);
        odometry.resetPosition(pose, Rotation2d.fromDegrees(getHeading()));
    }

    /**
     * Drives the robot using arcade controls.
     *
     * @param fwd the commanded forward movement
     * @param rot the commanded rotation
     */
    public void arcadeDrive(double fwd, double rot) {
        drive.arcadeDrive(fwd, rot);
    }

    /**
     * Controls the left and right sides of the drive directly with voltages.
     *
     * @param leftVolts the commanded left output
     * @param rightVolts the commanded right output
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotors.setVoltage(leftVolts);
        rightMotors.setVoltage(rightVolts);
        drive.feed();
    }

    /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }

    /**
     * Gets the average distance of the two encoders.
     *
     * @return the average of the two encoder readings
     */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    /**
     * Gets the left drive encoder.
     *
     * @return the left drive encoder
     */
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    /**
     * Gets the right drive encoder.
     *
     * @return the right drive encoder
     */
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    /**
     * Sets the max output of the drive. Useful for scaling the drive to drive more slowly.
     *
     * @param maxOutput the maximum output to which the drive will be constrained
     */
    public void setMaxOutput(double maxOutput) {
        drive.setMaxOutput(maxOutput);
    }

    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }

    /**
     * Returns the heading of the robot.
     *
     * @return the robot's heading in degrees, from -180 to 180
     */
    public double getHeading() {
        return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
    }

    public DifferentialDriveKinematics getKinematics() {
        return kinematics;
    }

    public Field2d getFieldSim() {
        return fieldSim;
    }
}