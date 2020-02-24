/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Intaking;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class IntakeAndStore extends ParallelCommandGroup {
  /**
   * Creates a new IntakeAndStore.
   */
  boolean isAuto = false;
  public IntakeAndStore(boolean isAuto) {
    addCommands(
      new IntakeIn(isAuto),
      new StoreBalls(isAuto)
    );
  }
}
