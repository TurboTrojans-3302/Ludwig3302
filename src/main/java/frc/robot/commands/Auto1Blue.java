// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Auto1Blue extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  Shooter m_shooter;
  Harvester m_harvester;
  DriveSubsystem m_robotDrive;


  public Auto1Blue() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
        new StartSpeaker(m_shooter, m_harvester),
        //Cross the line and go to note (8 (2.4384m) feet away exactly in auton)
        new GoToCommand(m_robotDrive, new Pose2d(2.3, 0, Rotation2d.fromDegrees(0)))
        //todo make harvester to floor and go to command be a parallel command group
       


    );
  }
}
