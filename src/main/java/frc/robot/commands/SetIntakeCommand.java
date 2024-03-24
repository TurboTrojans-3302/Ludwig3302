// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Harvester;

public class SetIntakeCommand extends Command {
  private Harvester harvester;
  private double speed;
  private double time;
  private Timer timer;

  /** Creates a new SetIntakeCommand. */
  public SetIntakeCommand(Harvester harvester, double speed, double time) {
    this.harvester = harvester;
    this.speed = speed;
    this.time = time;
    addRequirements(harvester);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    harvester.setIntakeSpeed(speed);
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    harvester.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
