// Copyright (c) FIRST and other WPILib contributors.

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.MotionProfileCommand;
import frc.robot.subsystems.DriveSubsystem;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer
{
    // The robot's subsystems and commands are defined here...
    private final DriveSubsystem subsystem = new DriveSubsystem();

    // private final DifferentialDrivetrainSim sim = new DifferentialDrivetrainSim(
    //         DCMotor.getNEO(3),
            
    // );

    TankMotionProfile prof = new TankMotionProfile(ParametricSpline.fromWaypoints(new Waypoint[]{
            new Waypoint(0, 0, 0, 2, 3),
            new Waypoint(9, 4, Math.toRadians(-90), 2, 5),
            new Waypoint(11, 4, Math.toRadians(90), 2, 5), 
            new Waypoint(13, 4, Math.toRadians(-90), 2, 5), 
            new Waypoint(15, 4, Math.toRadians(90), 2, 5), 
            new Waypoint(17, 4, Math.toRadians(-90), 2, 5), 
            new Waypoint(7.5, 11.0, Math.toRadians(-160), 3, 6),
            new Waypoint(0, 0, Math.toRadians(-90), 3, 5)
    }), new TankMotionProfile.TankMotionProfileConstraints(3, 3));

    // Trajectory t = TrajectoryGenerator.generateTrajectory(List.of(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(9.2, 4.6, new Rotation2d(0)), new Pose2d(21.6, 4.6, new Rotation2d(0))),
    //         new TrajectoryConfig(
    //                 10,
    //                 10
    //         ));

      private final MotionProfileCommand autoCommand = new MotionProfileCommand(subsystem, prof);
//   private final RamseteCommand autoCommand = new RamseteCommand(
//       t,
//       subsystem::getPose,
//       new RamseteController(),
//       new SimpleMotorFeedforward(
//           DriveConstants.ksVolts,
//           DriveConstants.kvVoltSecondsPerMeter,
//           DriveConstants.kaVoltSecondsSquaredPerMeter),
//       DriveConstants.kDriveKinematics,
//       subsystem::getWheelSpeeds,
//       new PIDController(DriveConstants.kPDriveVel, 0, 0),
//       new PIDController(DriveConstants.kPDriveVel, 0, 0),
//       // RamseteCommand passes volts to the callback
//       subsystem::tankDriveVolts,
//       subsystem);
    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer()
    {
        // Configure the button bindings
        configureButtonBindings();
        // subsystem.getFieldSim().getObject("trajectory").setTrajectory(prof.asTrajectory());
    }
    
    
    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings()
    {
        // Add button to command mappings here.
        // See https://docs.wpilib.org/en/stable/docs/software/commandbased/binding-commands-to-triggers.html
    }
    
    
    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand()
    {
        // An ExampleCommand will run in autonomous
        return autoCommand;
    }
}
