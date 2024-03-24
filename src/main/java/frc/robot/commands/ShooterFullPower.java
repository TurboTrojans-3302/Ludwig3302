// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterFullPower extends Command {
  private Shooter shooter;
  private double time;
  private Timer timer;
  private double rpm;


/** Creates a new ShooterFullPower. */
  public ShooterFullPower(Shooter shooter, double time, double rpm) {
    this.shooter = shooter;
    this.time = time;
    this.rpm = rpm;
    addRequirements(shooter);
    timer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.restart();
    shooter.setFullPower(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setRPM(rpm);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time) || shooter.mLeftVelocity >= rpm;
  }
}
