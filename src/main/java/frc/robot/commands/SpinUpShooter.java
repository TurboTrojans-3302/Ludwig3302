// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Harvester;
import frc.robot.subsystems.Shooter;

public class SpinUpShooter extends Command {
  Shooter mShooter;
  Harvester mHarvester;
  double targetRPM;
  /** Creates a new SpinUpShooter. */
  public SpinUpShooter(Shooter shooter, Harvester harvester, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mHarvester = harvester;
    targetRPM = rpm;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooter.setRPM(targetRPM);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mHarvester.setIntakeSpeed(Constants.harvesterConstants.outSpeed);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mShooter.speedIsReady();
  }
}
