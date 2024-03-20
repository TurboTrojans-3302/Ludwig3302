// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1Red extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  Shooter m_shooter;
  Harvester m_harvester;
  DriveSubsystem m_robotDrive;
  double ampAngle;
  Pose2d FromCenterStartToCenterRing;
  Pose2d FromCenterStartToAmp;


  public Auto1Red(DriveSubsystem drive, Shooter shooter, Harvester harvester) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    m_harvester = harvester;
    m_shooter = shooter;
    m_robotDrive = drive;
    ampAngle = Constants.harvesterConstants.ANGLE_AT_AMP;
    FromCenterStartToAmp = Constants.FieldConstants.FromCentrStartToAmpRed;
    FromCenterStartToCenterRing = Constants.FieldConstants.FromCentrStartToCentrRing;
    addCommands(
        new StartSpeaker(m_shooter, m_harvester)
        .andThen(Commands.parallel(new SetArmAngleCommand(m_harvester, Constants.harvesterConstants.ANGLE_AT_FLOOR), 
                                  new GoToCommand(m_robotDrive, FromCenterStartToCenterRing)))
        .andThen(new FloorPickUp(m_harvester, ampAngle))
        .andThen(new GoToCommand(m_robotDrive, FromCenterStartToAmp))
        //TODO completely the same except that x should be inverted from blue when I figure it out
        //1.872 meters to amp from center speaker
        //find sideways distance (y value)
        //Cross the line and go to note (8 (2.4384m) feet away exactly in auton)

        //GoToAndArmFloor(m_harvester, m_robotDrive, 2.3, 0.0, 0.0),
        //todo make harvester to floor and go to command be a parallel command group
       //new Intake(m_harvester, -1));

    );
  
  }
}
