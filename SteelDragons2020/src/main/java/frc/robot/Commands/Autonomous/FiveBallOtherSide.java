/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Commands.Autonomous;

import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Commands.Intaking.IntakeAndStore;
import frc.robot.Commands.Shooting.AutoShoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/latest/docs/software/commandbased/convenience-features.html
public class FiveBallOtherSide extends SequentialCommandGroup {
  /**
   * Creates a new FiveBallOtherSide.
   */
  public FiveBallOtherSide() {
    // Add your commands in the super() call, e.g.
    // super(new FooCommand(), new BarCommand());
    addCommands(
      new TubeBottom(),
      new ParallelRaceGroup(
        new AutoDrive(2.4, 0.55, true),
        new IntakeAndStore(true)
      ),
      new ParallelRaceGroup(
        new WaitTime(1.0),
        new IntakeAndStore(true)
      ),
      new ParallelRaceGroup(
        new AutoTurn(0.6, 0.45, false),
        new IntakeAndStore(true)
      ),
      new AutoDrive(1.3, 0.9, false),
      new WaitTime(0.15),
      new AutoTurn(0.5, 0.45, true),
      new AutoShoot(true)
    );
  }
}
