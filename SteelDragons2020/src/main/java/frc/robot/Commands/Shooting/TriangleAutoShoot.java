/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooting;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Commands.Autonomous.AutoDrive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TriangleAutoShoot extends SequentialCommandGroup {
  /**
   * Creates a new TriangleAutoShoot.
   */
  public TriangleAutoShoot() {
    addCommands(
      new ParallelCommandGroup(
        new AutoDrive(0.4, 0.7, true),
        new TubeTop(),
        new BeltDownForTime(false)

      ),
      new ParallelRaceGroup(
        new BeltUp(false, Constants.kButtonB, true, false),
        new Shoot(false, Constants.kButtonB, true, false)
      )
    );

  }
}
