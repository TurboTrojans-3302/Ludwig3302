// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.utils.TonePlayer;

public class Tones extends Command {
  private TonePlayer[] players;
  private int step = 0;
  private Timer timer = new Timer();


  private final Double[][] notes = {
    {697.0, 1477.0, 1.0},
    {  0.0,    0.0, 0.5},
    {697.0, 1477.0, 1.0},
    {  0.0,    0.0, 0.5},
    {941.0, 1336.0, 1.0},
    {  0.0,    0.0, 0.5},
    {697.0, 1336.0, 1.0}
  };

  /** Creates a new Tones. */
  public Tones(TonePlayer ... players) {
    this.players = players;
    addRequirements(players);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    step = 0;
    timer.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    for(TonePlayer p : players){
      p.setTones(notes[step][0], notes[step][1]);
    }

    if(timer.hasElapsed(notes[step][2])){
      step++;
      timer.restart();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    for(TonePlayer p : players){
      p.stop();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return step == notes.length;
  }
}
