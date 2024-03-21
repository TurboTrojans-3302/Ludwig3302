// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Harvester;
import frc.utils.SwerveUtils;

public class SetArmAngleCommand extends Command {
  Harvester mHarvester;
  Double mTargetAngle;

  /** Creates a new SetArmAngleCommand. */
  public SetArmAngleCommand(Harvester harvester, Double angle) {
    mHarvester = harvester;
    mTargetAngle = angle;
    addRequirements(mHarvester);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double setpoint = SwerveUtils.StepTowards(mHarvester.getArmAngle(),
                                              mTargetAngle,
                                              Constants.harvesterConstants.MAX_ARM_SPEED * Robot.kDefaultPeriod);
    mHarvester.setArmAngle(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mHarvester.isArmAtAngle();
  }
}
