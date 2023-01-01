// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.TankMotionProfile;
import frc.robot.subsystems.DriveSubsystem;


/** An example command that uses an example subsystem. */
public class MotionProfileCommand extends CommandBase
{
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveSubsystem subsystem;
    private RamseteController ramseteController;

    private final Timer timer;

    private TankMotionProfile motionProfile;
    private double maxTime; 

    private double startTime;
    private double prevTime;

    private Pose2d startPose;

    private PIDController ctrlLeft;
    private PIDController ctrlRight;
    private DifferentialDriveWheelSpeeds prevSpeeds;

    private SimpleMotorFeedforward feedforward;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public MotionProfileCommand(DriveSubsystem subsystem, TankMotionProfile motionProfile)
    {
        this.subsystem = subsystem;
        this.timer = new Timer(); 
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);

        this.subsystem.getFieldSim().getObject("trajectory").setTrajectory(motionProfile.asTrajectory());



        this.motionProfile = motionProfile;
       this.maxTime = motionProfile.getTotalTime();

        this.ramseteController = new RamseteController();

        this.feedforward = new SimpleMotorFeedforward(Constants.DriveConstants.ksVolts, Constants.DriveConstants.kvVoltSecondsPerMeter, Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);

        this.ctrlLeft = new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0);
        this.ctrlRight = new PIDController(Constants.DriveConstants.kPDriveVel, 0, 0);
    }
    
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        startPose = subsystem.getPose();
        prevTime = -1;
        timer.reset();
        timer.start();

        var initialState = this.motionProfile.getStateAtTime(0);
        prevSpeeds =
                this.subsystem.getKinematics().toWheelSpeeds(
                        new ChassisSpeeds(
                                initialState.velocityMetersPerSecond,
                                0,
                                initialState.curvatureRadPerMeter * initialState.velocityMetersPerSecond
                                ));

        subsystem.resetOdometry(initialState.poseMeters);
    }
    
    
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double curTime = timer.get();

        double dt = curTime - prevTime;
        Trajectory.State state = this.motionProfile.getStateAtTime(curTime);

        if (prevTime < 0) {
            subsystem.tankDriveVolts(0, 0);
            prevTime = curTime;
            return;
        }

        Transform2d transform2d = subsystem.getPose().minus(startPose);

        ChassisSpeeds speeds = ramseteController.calculate(
        //        new Pose2d(transform2d.getTranslation(), transform2d.getRotation())
                subsystem.getPose()
                , state);
        DifferentialDriveWheelSpeeds wheelSpeeds = subsystem.getKinematics().toWheelSpeeds(speeds);


        var leftSpeedSetpoint = wheelSpeeds.leftMetersPerSecond;
        var rightSpeedSetpoint = wheelSpeeds.rightMetersPerSecond;

        double leftFeedforward =
                feedforward.calculate(
                        leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.leftMetersPerSecond) / dt);

        double rightFeedforward =
                feedforward.calculate(
                        rightSpeedSetpoint, (rightSpeedSetpoint - prevSpeeds.rightMetersPerSecond) / dt);

        double leftOutput =
                leftFeedforward
                        + ctrlLeft.calculate(subsystem.getWheelSpeeds().leftMetersPerSecond, leftSpeedSetpoint);

        double rightOutput =
                rightFeedforward
                        + ctrlRight.calculate(
                        subsystem.getWheelSpeeds().rightMetersPerSecond, rightSpeedSetpoint);
        this.subsystem.tankDriveVolts(leftOutput, rightOutput);

        prevSpeeds = wheelSpeeds;
        prevTime = curTime;
    }
    
    
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
    
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished()
    {
        return timer.hasElapsed(maxTime);
    }
}
