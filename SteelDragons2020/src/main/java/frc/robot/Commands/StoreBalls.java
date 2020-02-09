/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Subsystems.Tube;

public class StoreBalls extends CommandBase {
  /**
   * Creates a new StoreBalls.
   */
  Tube tube;
  DigitalInput beamBreakSensor;
  boolean ball;
  double startTime = -1.0;

  public StoreBalls() {
    this.tube = RobotContainer.tube;
    beamBreakSensor = new DigitalInput(5);
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.tube);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.tube.beltStop();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!beamBreakSensor.get()) {
      ball = true;
      this.tube.beltUp();
      System.out.println("BALLLL");
    }
    else {
      if(ball) {
        startTime = Timer.getFPGATimestamp();
      }
      ball = false;
      if(Timer.getFPGATimestamp() >= 0.35 + startTime) {
        this.tube.beltStop();
      }
      System.out.println("NO BALLLL");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    this.tube.beltStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (!RobotContainer.operator.getRawButton(Constants.kButtonA));
  }
}
