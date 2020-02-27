/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Shooting;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class TriangleAutoShoot extends SequentialCommandGroup {
  /**
   * Creates a new TriangleAutoShoot.
   */
  boolean isAuto;
  public TriangleAutoShoot(boolean isAuto) {
    this.isAuto = isAuto;
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());

    new ParallelRaceGroup(
      new BeltUp(this.isAuto),
      new Shoot()
    );
  }
}
