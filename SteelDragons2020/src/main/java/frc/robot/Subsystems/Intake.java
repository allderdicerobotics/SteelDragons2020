/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.RobotContainer;


public class Intake extends SubsystemBase {
  private final CANSparkMax intakeFoldMotor;
  private CANPIDController intakeFoldMotorPIDController;
  private CANEncoder intakeFoldMotorCANEncoder;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;

  private final CANSparkMax intakeSpinMotor;

  
  public Intake() {
    intakeFoldMotor = new CANSparkMax(Constants.CAN_FOLD_INTAKE, MotorType.kBrushless);
    intakeFoldMotorPIDController = intakeFoldMotor.getPIDController();
    intakeFoldMotorCANEncoder = intakeFoldMotor.getEncoder();

    intakeSpinMotor = new CANSparkMax(Constants.CAN_SPIN_INTAKE, MotorType.kBrushless);

    kP = 0.00005;
    kI = 0.0;
    kD = 0.0;
    kIz = 0.0;
    kFF = 0.00005;
    kMaxOutput = 1.0;
    kMinOutput = -1.0;
    maxRPM = 5700;

    maxVel = 5000;
    maxAcc = 10000;

    intakeFoldMotorPIDController.setP(kP);
    intakeFoldMotorPIDController.setI(kI);
    intakeFoldMotorPIDController.setD(kD);
    intakeFoldMotorPIDController.setIZone(kIz);
    intakeFoldMotorPIDController.setFF(kFF);
    intakeFoldMotorPIDController.setOutputRange(kMinOutput, kMaxOutput);

    int smartMotionSlot = 0;
    intakeFoldMotorPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    intakeFoldMotorPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    intakeFoldMotorPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    intakeFoldMotorPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // //Use this only for tuning PID values and testing
    // SmartDashboard.putNumber("IntakeFold P Gain", kP);
    // SmartDashboard.putNumber("IntakeFold I Gain", kI);
    // SmartDashboard.putNumber("IntakeFold D Gain" , kD);
    // SmartDashboard.putNumber("IntakeFold I Zone", kIz);
    // SmartDashboard.putNumber("IntakeFold Feed Forward", kFF);
    // SmartDashboard.putNumber("IntakeFold Max Output", kMaxOutput);
    // SmartDashboard.putNumber("IntakeFold Min Output", kMinOutput);

    // SmartDashboard.putNumber("IntakeFold Max Velocity", maxVel);
    // SmartDashboard.putNumber("IntakeFold Min Velocity", minVel);
    // SmartDashboard.putNumber("IntakeFold Max Acceleration", maxAcc);
    // SmartDashboard.putNumber("IntakeFold Allowed Closed Loop Error", allowedErr);
    // SmartDashboard.putNumber("IntakeFold Set Position", 0);
    // SmartDashboard.putNumber("IntakeFold Set Velocity", 0);

    // // button to toggle between velocity and smart motion modes
    // SmartDashboard.putBoolean("IntakeFold Mode", true);
  }

  //INTAKE SPIN
  public void spinIn() {
    setSpinSpeed(-Constants.INTAKE_SPEED_IN);
  }

  public void spinOut() {
    setSpinSpeed(Constants.INTAKE_SPEED_OUT);
  }

  public void spinStop() {
    setSpinSpeed(0.0);
  }

  public void spinOnDriveSpeed(boolean isAuto) {
    if(isAuto) {
      setSpinSpeed(0.8);
    } else {
      setSpinSpeed(-0.6 - (0.4 *
      Math.abs(RobotContainer.driver.getRawAxis(Constants.kLeftStickY))));
    }
  }

  public void setSpinSpeed(double speed) {
    intakeSpinMotor.set(speed);
  }

  //INTAKE POSITION
  public void bottom() {
    setPosition(-15.0);
  }

  public void top() {
    setPosition(0.0);
  }

  public void setPosition(double position){
    intakeFoldMotorPIDController.setReference(position, ControlType.kSmartMotion);
  }

  public void PIDSetup() {
    // // read PID coefficients from SmartDashboard
    // double p = SmartDashboard.getNumber("IntakeFold P Gain", 0);
    // double i = SmartDashboard.getNumber("IntakeFold I Gain", 0);
    // double d = SmartDashboard.getNumber("IntakeFold D Gain", 0);
    // double iz = SmartDashboard.getNumber("IntakeFold I Zone", 0);
    // double ff = SmartDashboard.getNumber("IntakeFold Feed Forward", 0);
    // double max = SmartDashboard.getNumber("IntakeFold Max Output", 0);
    // double min = SmartDashboard.getNumber("IntakeFold Min Output", 0);
    // double maxV = SmartDashboard.getNumber("IntakeFold Max Velocity", 0);
    // double minV = SmartDashboard.getNumber("IntakeFold Min Velocity", 0);
    // double maxA = SmartDashboard.getNumber("IntakeFold Max Acceleration", 0);
    // double allE = SmartDashboard.getNumber("IntakeFold Allowed Closed Loop Error", 0);

    // // if PID coefficients on SmartDashboard have changed, write new values to controller
    // if((p != kP)) { intakeFoldMotorPIDController.setP(p); kP = p; }
    // if((i != kI)) { intakeFoldMotorPIDController.setI(i); kI = i; }
    // if((d != kD)) { intakeFoldMotorPIDController.setD(d); kD = d; }
    // if((iz != kIz)) { intakeFoldMotorPIDController.setIZone(iz); kIz = iz; }
    // if((ff != kFF)) { intakeFoldMotorPIDController.setFF(ff); kFF = ff; }
    // if((max != kMaxOutput) || (min != kMinOutput)) { 
    //   intakeFoldMotorPIDController.setOutputRange(min, max); 
    //   kMinOutput = min; kMaxOutput = max; 
    // }
    // if((maxV != maxVel)) { intakeFoldMotorPIDController.setSmartMotionMaxVelocity(maxV,0); maxVel = maxV; }
    // if((minV != minVel)) { intakeFoldMotorPIDController.setSmartMotionMinOutputVelocity(minV,0); minVel = minV; }
    // if((maxA != maxAcc)) { intakeFoldMotorPIDController.setSmartMotionMaxAccel(maxA,0); maxAcc = maxA; }
    // if((allE != allowedErr)) { intakeFoldMotorPIDController.setSmartMotionAllowedClosedLoopError(allE,0); allowedErr = allE; }

    // double setPoint, processVariable;
    // boolean mode = SmartDashboard.getBoolean("IntakeFold Mode", false);
    // if(mode) {
    //   setPoint = SmartDashboard.getNumber("IntakeFold Set Velocity", 0);
    //   intakeFoldMotorPIDController.setReference(setPoint, ControlType.kVelocity);
    //   processVariable = intakeFoldMotorCANEncoder.getVelocity();
    // } else {
    //   setPoint = SmartDashboard.getNumber("IntakeFold Set Position", 0);
    //   /**
    //    * As with other PID modes, Smart Motion is set by calling the
    //    * setReference method on an existing pid object and setting
    //    * the control type to kSmartMotion
    //    */
    //   intakeFoldMotorPIDController.setReference(setPoint, ControlType.kSmartMotion);
    //   processVariable = intakeFoldMotorCANEncoder.getPosition();
    // }

    // SmartDashboard.putNumber("IntakeFold SetPoint", setPoint);
    // SmartDashboard.putNumber("IntakeFold Process Variable", processVariable);
    // SmartDashboard.putNumber("IntakeFold Output", intakeFoldMotor.getAppliedOutput());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

