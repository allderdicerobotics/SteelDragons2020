/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.TeleopDrive;
import frc.robot.Commands.Autonomous.DoNothing;
import frc.robot.Commands.Autonomous.PositionLeft;
import frc.robot.Commands.Autonomous.PositionMiddle;
import frc.robot.Commands.Autonomous.PositionRight;
import frc.robot.Commands.Intaking.IntakeAndStore;
import frc.robot.Commands.Shooting.AutoShoot;
import frc.robot.Subsystems.ClimbingElevators;
import frc.robot.Subsystems.ClimbingWinch;
import frc.robot.Subsystems.DriveTrain;
import frc.robot.Subsystems.Intake;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Tube;
import frc.robot.Subsystems.TubeBelts;


public class RobotContainer {
    public static final DriveTrain driveTrain = new DriveTrain();
    public static final Tube tube = new Tube();
    public static final Intake intake = new Intake();
    public static final Shooter shooter = new Shooter();
    public static final TubeBelts tubeBelts = new TubeBelts();
    public static final ClimbingElevators climbingElevators = new ClimbingElevators();
    public static final ClimbingWinch climbingWinch = new ClimbingWinch();


    public static final Joystick driver = new Joystick(0);
    public static final Joystick operator = new Joystick(1);

    private static final String kDefaultAuto = "Do Nothing";
    private static final String kPositionLeft = "PositionLeft";
    private static final String kPositionMiddle = "PositionMiddle";
    private static final String kPositionRight = "PositionRight";

    public static int currentBallCount = 3;

    private String autoSelected;
    private final SendableChooser<String> autoChooser = new SendableChooser<>();

    
    public Command getAutonomousCommand() {
        autoSelected = autoChooser.getSelected();
        System.out.println("Auto selected: " + autoSelected);
        Command returnCommand;
        switch(autoSelected) {
            case "PositionLeft":
                returnCommand = (new PositionLeft());
                break;
            case "PositionMiddle":
                returnCommand = (new PositionMiddle());
                break;
            case "PositionRight":
                returnCommand = (new PositionRight());
                break;
            default:
                returnCommand = (new DoNothing());
                break;
        }
        return returnCommand;
    }

    public RobotContainer() {
        configureButtonBindings();
        
        driveTrain.setDefaultCommand(new TeleopDrive(driveTrain));

        autoChooser.setDefaultOption("Do Nothing", kDefaultAuto);
        autoChooser.addOption("PositionLeft", kPositionLeft);
        autoChooser.addOption("PositionMiddle", kPositionMiddle);
        autoChooser.addOption("PositionRight", kPositionRight);
        SmartDashboard.putData("Auto Chooser", autoChooser);

        SmartDashboard.putNumber("Ball Count", currentBallCount);
    }

    public void configureButtonBindings() {
        new JoystickButton(driver, Constants.kButtonA)
            .whenPressed(new AutoShoot(false));

        new JoystickButton(driver, Constants.kButtonBack)
            .whenPressed(() -> shooter.setColorWheelFastSpeed())
            .whenReleased(() -> shooter.stop());

        new JoystickButton(driver, Constants.kButtonStart)
            .whenPressed(() -> shooter.setColorWheelSlowSpeed())
            .whenReleased(() -> shooter.stop());

        //BELTS
        new JoystickButton(operator, Constants.kMiddleRight)
            .whenPressed(() -> tubeBelts.up())
            .whenReleased(() -> tubeBelts.stop());
        new JoystickButton(operator, Constants.kMiddleMiddleRight)
            .whenPressed(() -> tubeBelts.down())
            .whenReleased(() -> tubeBelts.stop());

        //SHOOTER
        new JoystickButton(operator, Constants.kBottomMiddleLeft)
            .whenPressed(() -> shooter.normalSpeed())
            .whenReleased(() -> shooter.stop());

        //INTAKE SPIN
        new JoystickButton(operator, Constants.kBottomRight)
            .whenPressed(new IntakeAndStore(false));
        new JoystickButton(operator, Constants.kBottomMiddleRight)
            .whenPressed(() -> intake.spinOut())
            .whenReleased(() -> intake.spinStop());

        //INTAKE POSITION
        // new JoystickButton(operator, Constants.kMiddleMiddleLeft)
        //     .whenPressed(() -> intake.bottom());
        // new JoystickButton(operator, Constants.kTopMiddleLeft)
        //     .whenPressed(() -> intake.top());
 
        //TUBE MANUAL
        Trigger consoleTriggerYUp = new AxisButton(operator, Constants.kTopMiddleRightYAxis, true);
            consoleTriggerYUp.whileActiveContinuous(() -> tube.up());

        Trigger consoleTriggerYDown = new AxisButton(operator, Constants.kTopMiddleRightYAxis, false);
            consoleTriggerYDown.whileActiveContinuous(() -> tube.down());

        //CLIMBER
        new JoystickButton(operator, Constants.kMiddleMiddleLeft)
            .whenPressed(() -> climbingElevators.goUp())
            .whenReleased(() -> climbingElevators.stop());
        new JoystickButton(operator, Constants.kTopMiddleLeft)
            .whenPressed(() -> climbingWinch.winchDown())
            .whenReleased(() -> climbingWinch.stop());
    }

    public static double[] getLimeLightValues() {
        double[] values = new double[4];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
        NetworkTableEntry tx = table.getEntry("tx");
        NetworkTableEntry ty = table.getEntry("ty");
        NetworkTableEntry ta = table.getEntry("ta");
        NetworkTableEntry tv = table.getEntry("tv");
        
        //read values periodically
        values[0] = tv.getDouble(0.0);
        values[1] = tx.getDouble(0.0);
        values[2] = ty.getDouble(0.0);
        values[3] = ta.getDouble(0.0);
        return values;
      }
    
    public static double getDistanceFromTarget() {
        double currentYPosition = getLimeLightValues()[2];
        return (78.0/Math.tan(Math.toRadians(32.5 + currentYPosition)));
    }

    public static void addOneBall() {
        currentBallCount++;
    }

    public static void getRidOfAllBalls() {
        currentBallCount = 0;
    }

    public static int getBallCount() {
        return currentBallCount;
    }

    public static double[] getRaspberryValues() {
        double[] values = new double[4];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("FRCvisionpc");
        NetworkTableEntry tx = table.getEntry("pi_tx");
        NetworkTableEntry ty = table.getEntry("pi_ty");
        NetworkTableEntry ta = table.getEntry("pi_ta");
        
        //read values periodically
        values[1] = tx.getDouble(0.0);
        values[2] = ty.getDouble(0.0);
        values[3] = ta.getDouble(0.0);
        values[0] = ((values[1]==-1)&&(values[2]==-1))?0:1;
        return values;
      }
}
