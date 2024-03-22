// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SpeakerScoreTwiceStageSideRed extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  Shooter m_shooter;
  Harvester m_harvester;
  DriveSubsystem m_robotDrive;
  Pose2d FromCenterStartToCenterRing;
  Pose2d SpeakerPos;
  Double speakerAngle;


  public SpeakerScoreTwiceStageSideRed(DriveSubsystem drive, Shooter shooter, Harvester harvester) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    speakerAngle = Constants.harvesterConstants.ANGLE_AT_SPEAKER;
    SpeakerPos = Constants.FieldConstants.CentrBackToSpeaker;
    FromCenterStartToCenterRing = Constants.FieldConstants.FromCentrStartToCentrRing;
    m_harvester = harvester;
    m_shooter = shooter;
    m_robotDrive = drive;
    addCommands(
        new StartPositionCommand(m_robotDrive, new Pose2d(0,0,Rotation2d.fromDegrees(45))),
        new ShootCommand(m_shooter, m_harvester),
        new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_FLOOR), 
        new SetIntakeSpeed(m_harvester, 1.0),
        new GoToCommand(m_robotDrive, new Pose2d(0.0, 0.3, Rotation2d.fromDegrees(0))),
        new GoToCommand(m_robotDrive, new Pose2d(2.438, 0.3, Rotation2d.fromDegrees(0))),
        new WaitCommand(0.5),
        new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_DRIVE),
      
        new GoToCommand(m_robotDrive, new Pose2d(0.0, 0.3, Rotation2d.fromDegrees(0))),
        new SetIntakeSpeed(m_harvester, 0.0),
        new GoToCommand(m_robotDrive, new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(45))),
        new ShootCommand(m_shooter, m_harvester)


      

        
        //1.872 meters to amp from center speaker
        //find sideways distance (y value)
        //Cross the line and go to note (8 (2.4384m) feet away exactly in auton)

        //GoToAndArmFloor(m_harvester, m_robotDrive, 2.3, 0.0, 0.0),
        //todo make harvester to floor and go to command be a parallel command group
       //new Intake(m_harvester, -1));

    );
  
  }
}
