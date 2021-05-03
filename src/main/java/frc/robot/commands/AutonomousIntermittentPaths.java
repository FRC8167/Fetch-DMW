// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

public class AutonomousIntermittentPaths extends SequentialCommandGroup {

    final Rotation2d rotation180 = Rotation2d.fromDegrees(180.0);
    final Rotation2d rotation270 = Rotation2d.fromDegrees(270.0);
    final Rotation2d rotation90 = Rotation2d.fromDegrees(90.0);
    final Rotation2d rotation0 = Rotation2d.fromDegrees(0.0);

    //private final Drivetrain m_drivetrain = new Drivetrain();
    
    // Define the initial pose to be used by this command. This will be used in the initial trajectory
    // and will allow the system to query for it
    //private Pose2d initialPose = new Pose2d(0,0, rotation0);
    //private Trajectory trajectorypath1;
    //private Trajectory trajectorypath2;
    //private ArrayList<Translation2d> waypointList = new ArrayList<Translation2d>();
  /** Creates a new AutonomousIntermittentPaths. */
  public AutonomousIntermittentPaths(Drivetrain m_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    
      m_drivetrain.arcadeDrive(0, 0);
      m_drivetrain.resetEncoders();
     // m_drivetrain.resetOdometry(null);

    var autoVoltageConstraint =
    new DifferentialDriveVoltageConstraint(
    new SimpleMotorFeedforward(DriveConstants.ksVolts, 
        DriveConstants.kvVoltSecondsPerMeter, 
        DriveConstants.kaVoltSecondsSquaredPerMeter),
        DriveConstants.kDriveKinematics,
        10);

    TrajectoryConfig configForward =
      new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
        AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        .setKinematics(DriveConstants.kDriveKinematics)
        .addConstraint(autoVoltageConstraint)
        .setReversed(false);

    TrajectoryConfig configBackward =
      new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
          AutoConstants.kMaxAccelerationMetersPerSecondSquared)
          .setKinematics(DriveConstants.kDriveKinematics)
          .addConstraint(autoVoltageConstraint)
          .setReversed(true);

    Trajectory trajectorypath1 = TrajectoryGenerator.generateTrajectory(
            new Pose2d(Units.inchesToMeters(0), Units.inchesToMeters(0), new Rotation2d(Units.degreesToRadians(0))),
                List.of(
                  new Translation2d(Units.inchesToMeters(10), Units.inchesToMeters(2))
                  //new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(4)),
                  //new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(9)),
              //new Translation2d(Units.inchesToMeters(13), Units.inchesToMeters(11))
                  ),
            new Pose2d(Units.inchesToMeters(13.5), Units.inchesToMeters(14.5), new Rotation2d(Units.degreesToRadians(90))),
            configForward);  

      Trajectory trajectorypath2 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(13.5), Units.inchesToMeters(14.5), new Rotation2d(Units.degreesToRadians(90))),
            List.of(
              new Translation2d(Units.inchesToMeters(18), Units.inchesToMeters(0)),
              //new Translation2d(Units.inchesToMeters(24), Units.inchesToMeters(-7.5)),
              new Translation2d(Units.inchesToMeters(34), Units.inchesToMeters(-15))
              //new Translation2d(Units.inchesToMeters(32), Units.inchesToMeters(-12))
              ),
        new Pose2d(Units.inchesToMeters(37.5), Units.inchesToMeters(16), new Rotation2d(Units.degreesToRadians(280))),
        configBackward);  

      Trajectory trajectorypath3 = TrajectoryGenerator.generateTrajectory(
        new Pose2d(Units.inchesToMeters(37.5), Units.inchesToMeters(16), new Rotation2d(Units.degreesToRadians(280))),
              List.of(
                //new Translation2d(Units.inchesToMeters(36), Units.inchesToMeters(0)),
                //new Translation2d(Units.inchesToMeters(38), Units.inchesToMeters(-7.5)),
                new Translation2d(Units.inchesToMeters(58), Units.inchesToMeters(-15))
                //new Translation2d(Units.inchesToMeters(58), Units.inchesToMeters(0))
               // new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(12))
                 ),
        new Pose2d(Units.inchesToMeters(61), Units.inchesToMeters(14.5), new Rotation2d(Units.degreesToRadians(90))),

                configForward); 


        Trajectory trajectorypath4 = TrajectoryGenerator.generateTrajectory(
          new Pose2d(Units.inchesToMeters(61), Units.inchesToMeters(14.5), new Rotation2d(Units.degreesToRadians(90))),
                        List.of(
                          //new Translation2d(Units.inchesToMeters(36), Units.inchesToMeters(0)),
                          //new Translation2d(Units.inchesToMeters(38), Units.inchesToMeters(-7.5)),
                          new Translation2d(Units.inchesToMeters(62), Units.inchesToMeters(-3))
                          //new Translation2d(Units.inchesToMeters(58), Units.inchesToMeters(0))
                         // new Translation2d(Units.inchesToMeters(60), Units.inchesToMeters(12))
                           ),
            new Pose2d(Units.inchesToMeters(78), Units.inchesToMeters(-9), new Rotation2d(Units.degreesToRadians(120))),
          
                          configBackward); 

    RamseteCommand Path1 = new RamseteCommand(
                  trajectorypath1,
                  m_drivetrain::getPose,
                  new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                  new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
                  DriveConstants.kDriveKinematics,
                  m_drivetrain::getWheelSpeeds,
                  new PIDController(DriveConstants.kPDriveVel, 0, 0),
                  new PIDController(DriveConstants.kPDriveVel, 0, 0),
                  m_drivetrain::tankDriveVolts,
                  m_drivetrain);
    
    RamseteCommand Path2 = new RamseteCommand(
                  trajectorypath2,
                  m_drivetrain::getPose,
                  new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                  new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
                  DriveConstants.kDriveKinematics,
                  m_drivetrain::getWheelSpeeds,
                  new PIDController(DriveConstants.kPDriveVel, 0, 0),
                  new PIDController(DriveConstants.kPDriveVel, 0, 0),
                  m_drivetrain::tankDriveVolts,
                  m_drivetrain);

    RamseteCommand Path3 = new RamseteCommand(
                    trajectorypath3,
                    m_drivetrain::getPose,
                    new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                    new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
                    DriveConstants.kDriveKinematics,
                    m_drivetrain::getWheelSpeeds,
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    new PIDController(DriveConstants.kPDriveVel, 0, 0),
                    m_drivetrain::tankDriveVolts,
                    m_drivetrain);

                    RamseteCommand Path4 = new RamseteCommand(
                      trajectorypath4,
                      m_drivetrain::getPose,
                      new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
                      new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
                      DriveConstants.kDriveKinematics,
                      m_drivetrain::getWheelSpeeds,
                      new PIDController(DriveConstants.kPDriveVel, 0, 0),
                      new PIDController(DriveConstants.kPDriveVel, 0, 0),
                      m_drivetrain::tankDriveVolts,
                      m_drivetrain);

                  addCommands(            
                   new SequentialCommandGroup(
                   Path1.andThen(() -> m_drivetrain.tankDriveVolts(0, 0)),
                  // new TurnDegrees(-0.5, Units.degreesToRadians(5), m_drivetrain),
                   //new TurnDegrees(0.5, Units.degreesToRadians(5), m_drivetrain),
                   Path2.andThen(() -> m_drivetrain.tankDriveVolts(0, 0)),
                   Path3.andThen(() -> m_drivetrain.tankDriveVolts(0, 0)),
                   Path4.andThen(() -> m_drivetrain.tankDriveVolts(0, 0))

                                 


                  //new DriveDistance(0.75, Units.inchesToMeters(22.5), m_drivetrain)
                   // Path3.andThen(() -> m_drivetrain.tankDriveVolts(0, 0))
                    )
                );
      
      
    
  }

  /* // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  } */
}
