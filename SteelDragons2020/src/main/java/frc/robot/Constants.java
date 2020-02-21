/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


public class Constants {
	//CAN IDs
    public static final int CAN_DRIVETRAIN_FRONTLEFT = 2;
    public static final int CAN_DRIVETRAIN_BACKLEFT = 6;
    public static final int CAN_DRIVETRAIN_FRONTRIGHT = 5;
    public static final int CAN_DRIVETRAIN_BACKRIGHT = 1;

    public static final int CAN_TUBE = 9;
    public static final int CAN_SHOOTER_LEFT = 10;
    public static final int CAN_SHOOTER_RIGHT = 7;

	public static final int CAN_FOLD_INTAKE = 11;
	public static final int CAN_SPIN_INTAKE = 8;

	public static final int CAN_BELT_LEFT = 12;
	public static final int CAN_BELT_RIGHT = 14;

	//SPEEDS
    public static final double DRIVETRAIN_THROTTLE_SCALE = 0.8;
    public static final double DRIVETRAIN_STEER_SCALE = 0.6;

    public static final double INTAKE_SPEED_IN = -0.8;
	public static final double INTAKE_SPEED_OUT = 0.8;

	public static final double BELT_SPEED_UP = 0.8;
	public static final double BELT_SPEED_DOWN = 0.8;

	public static final double SHOOTER_SPEED = 1.0;

	public static final double TUBE_SPEED_UP = 0.9;
	public static final double TUBE_SPEED_DOWN = 0.4;

	//OTHER
	public static final double TUBE_MANUAL_CHANGE = 0.8;
	
	public static final double AUTO_INTAKING_EXTRA_BELT_RUN_TIME = 0.35;

    //Logitech f310
	/* axis mappings */
	public static final int kLeftStickX = 0;
	public static final int kLeftStickY = 1;
	public static final int kLeftTrigger = 2;
	public static final int kRightTrigger = 3;
	public static final int kRightStickX = 4;
	public static final int kRightStickY = 5;
	
	/* button mappings */
	public static final int kButtonA = 1;
	public static final int kButtonB = 2;
	public static final int kButtonX = 3;
	public static final int kButtonY = 4;
	public static final int kButtonLeftBumper = 5;
	public static final int kButtonRightBumper = 6;
	public static final int kButtonBack = 7;
	public static final int kButtonStart = 8;
	public static final int kButtonLeftStick = 9;
	public static final int kButtonRightStick = 10;
	public static final int kPOVDPad = 0;

	//OPERATOR CONSOLE
	//k(ROW)(COLUMN)
	public static final int kTopMiddleLeft = 6;

	public static final int kTopMiddleRightYAxis = 1;
	public static final int kTopMiddleRightXAxis = 2;

	public static final int kTopRight = 2;
	public static final int kMiddleLeft = 11;
	public static final int kMiddleMiddleLeft = 5;
	public static final int kMiddleMiddleRight = 4;
	public static final int kMiddleRight = 3;
	public static final int kBottomLeft = 12;
	public static final int kBottomMiddleLeft = 8;
	public static final int kBottomMiddleRight = 7;
	public static final int kBottomRight = 1;
}
