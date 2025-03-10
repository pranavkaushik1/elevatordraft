// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorMove extends Command {

  private final Elevator elevator;

  /** Creates a new ElevatorMove. */
  public ElevatorMove() {
    // Use addRequirements() here to declare subsystem dependencies.
    this.elevator = elevator;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double targetPosition = 0.0;
    double currentPosition = elevator.getEncoderPosition();

    //  if button press is to go to mid
    if () {
      targetPosition = MID_POSITION;
      elevator.setMid(currentPosition);

      // if button press was to score
    } else if () {
        targetPosition = SCORING_POSITION;
        elevator.setSP(currentPosition);

        // if button press was to reset the elevator
    } else if () {
        elevator.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
