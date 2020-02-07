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

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private final CANSparkMax intakeFoldMotor;
  private CANPIDController intakeFoldMotorPIDController;
  private CANEncoder intakeFoldMotorCANEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private double currentIntakeFoldPosition;

  private final CANSparkMax intakeSpinMotor;

  public Intake() {
    intakeFoldMotor = new CANSparkMax(Constants.CAN_FOLD_INTAKE, MotorType.kBrushless);
    intakeSpinMotor = new CANSparkMax(Constants.CAN_SPIN_INTAKE, MotorType.kBrushless);

    /**
     * 
     * The RestoreFactoryDefaults method can be used to reset the configurations
     * parameters
     * 
     * in the SPARK MAX to their factory default state. If no argument is passed,
     * these
     * 
     * parameters will not persist between power cycles
     * 
     */

    intakeFoldMotor.restoreFactoryDefaults();

    // initialze PID controller and encoder objects

    intakeFoldMotorPIDController = intakeFoldMotor.getPIDController();

    intakeFoldMotorCANEncoder = intakeFoldMotor.getEncoder();

    // PID coefficients

    kP = 5e-5;

    kI = 1e-6;

    kD = 0;

    kIz = 0;

    kFF = 0.000156;

    kMaxOutput = 1;

    kMinOutput = -1;

    maxRPM = 5700;

    // Smart Motion Coefficients

    maxVel = 2000; // rpm

    maxAcc = 1500;

    // set PID coefficients

    intakeFoldMotorPIDController.setP(kP);

    intakeFoldMotorPIDController.setI(kI);

    intakeFoldMotorPIDController.setD(kD);

    intakeFoldMotorPIDController.setIZone(kIz);

    intakeFoldMotorPIDController.setFF(kFF);

    intakeFoldMotorPIDController.setOutputRange(kMinOutput, kMaxOutput);

    /**
     * 
     * Smart Motion coefficients are set on a CANPIDController object
     * 
     * 
     * 
     * - setSmartMotionMaxVelocity() will limit the velocity in RPM of
     * 
     * the pid controller in Smart Motion mode
     * 
     * - setSmartMotionMinOutputVelocity() will put a lower bound in
     * 
     * RPM of the pid controller in Smart Motion mode
     * 
     * - setSmartMotionMaxAccel() will limit the acceleration in RPM^2
     * 
     * of the pid controller in Smart Motion mode
     * 
     * - setSmartMotionAllowedClosedLoopError() will set the max allowed
     * 
     * error for the pid controller in Smart Motion mode
     * 
     */

    int smartMotionSlot = 0;

    intakeFoldMotorPIDController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);

    intakeFoldMotorPIDController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);

    intakeFoldMotorPIDController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);

    intakeFoldMotorPIDController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    // display PID coefficients on SmartDashboard

    SmartDashboard.putNumber("P Gain", kP);

    SmartDashboard.putNumber("I Gain", kI);

    SmartDashboard.putNumber("D Gain", kD);

    SmartDashboard.putNumber("I Zone", kIz);

    SmartDashboard.putNumber("Feed Forward", kFF);

    SmartDashboard.putNumber("Max Output", kMaxOutput);

    SmartDashboard.putNumber("Min Output", kMinOutput);

    // display Smart Motion coefficients

    SmartDashboard.putNumber("Max Velocity", maxVel);

    SmartDashboard.putNumber("Min Velocity", minVel);

    SmartDashboard.putNumber("Max Acceleration", maxAcc);

    SmartDashboard.putNumber("Allowed Closed Loop Error", allowedErr);

    SmartDashboard.putNumber("Set Position", 0);

    SmartDashboard.putNumber("Set Velocity", 0);

    // button to toggle between velocity and smart motion modes

    SmartDashboard.putBoolean("Mode", true);
  }

  // INTAKE SPIN
  public void spinIn() {
    setSpinSpeed(Constants.INTAKE_SPEED_IN);
  }

  public void spinOut() {
    setSpinSpeed(Constants.INTAKE_SPEED_OUT);
  }

  public void spinStop() {
    setSpinSpeed(0.0);
  }

  public void setSpinSpeed(double speed) {
    intakeSpinMotor.set(speed);
  }

  // INTAKE FOLD SPEEDS
  public void goUp() {
    intakeFoldMotor.set(0.1);
  }

  public void goDown() {
    intakeFoldMotor.set(-0.1);
  }

  public void stop() {
    intakeFoldMotor.set(0.0);
  }

  // INTAKE FOLD POSITIONS
  public void bottom() {
    setPosition(0.0);
  }

  public void top() {
    setPosition(300.0);
  }

  public void setPosition(double position) {
    intakeFoldMotorPIDController.setReference(position, ControlType.kSmartMotion);
    currentIntakeFoldPosition = position;
  }

  public void runPID() {
    // read PID coefficients from SmartDashboard

    double p = SmartDashboard.getNumber("P Gain", 0);

    double i = SmartDashboard.getNumber("I Gain", 0);

    double d = SmartDashboard.getNumber("D Gain", 0);

    double iz = SmartDashboard.getNumber("I Zone", 0);

    double ff = SmartDashboard.getNumber("Feed Forward", 0);

    double max = SmartDashboard.getNumber("Max Output", 0);

    double min = SmartDashboard.getNumber("Min Output", 0);

    double maxV = SmartDashboard.getNumber("Max Velocity", 0);

    double minV = SmartDashboard.getNumber("Min Velocity", 0);

    double maxA = SmartDashboard.getNumber("Max Acceleration", 0);

    double allE = SmartDashboard.getNumber("Allowed Closed Loop Error", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to
    // controller

    if ((p != kP)) {
      intakeFoldMotorPIDController.setP(p);
      kP = p;
    }

    if ((i != kI)) {
      intakeFoldMotorPIDController.setI(i);
      kI = i;
    }

    if ((d != kD)) {
      intakeFoldMotorPIDController.setD(d);
      kD = d;
    }

    if ((iz != kIz)) {
      intakeFoldMotorPIDController.setIZone(iz);
      kIz = iz;
    }

    if ((ff != kFF)) {
      intakeFoldMotorPIDController.setFF(ff);
      kFF = ff;
    }

    if ((max != kMaxOutput) || (min != kMinOutput)) {

      intakeFoldMotorPIDController.setOutputRange(min, max);

      kMinOutput = min;
      kMaxOutput = max;

    }

    if ((maxV != maxVel)) {
      intakeFoldMotorPIDController.setSmartMotionMaxVelocity(maxV, 0);
      maxVel = maxV;
    }

    if ((minV != minVel)) {
      intakeFoldMotorPIDController.setSmartMotionMinOutputVelocity(minV, 0);
      minVel = minV;
    }

    if ((maxA != maxAcc)) {
      intakeFoldMotorPIDController.setSmartMotionMaxAccel(maxA, 0);
      maxAcc = maxA;
    }

    if ((allE != allowedErr)) {
      intakeFoldMotorPIDController.setSmartMotionAllowedClosedLoopError(allE, 0);
      allowedErr = allE;
    }

    double setPoint, processVariable;

    boolean mode = SmartDashboard.getBoolean("Mode", false);

    if (mode) {

      setPoint = SmartDashboard.getNumber("Set Velocity", 0);

      intakeFoldMotorPIDController.setReference(setPoint, ControlType.kVelocity);

      processVariable = intakeFoldMotorCANEncoder.getVelocity();

    } else {

      setPoint = SmartDashboard.getNumber("Set Position", 0);

      /**
       * 
       * As with other PID modes, Smart Motion is set by calling the
       * 
       * setReference method on an existing pid object and setting
       * 
       * the control type to kSmartMotion
       * 
       */

      intakeFoldMotorPIDController.setReference(setPoint, ControlType.kSmartMotion);

      processVariable = intakeFoldMotorCANEncoder.getPosition();

    }

    SmartDashboard.putNumber("SetPoint", setPoint);

    SmartDashboard.putNumber("Process Variable", processVariable);

    SmartDashboard.putNumber("Output", intakeFoldMotor.getAppliedOutput());

  }

  @Override
  public void periodic() {
    runPID();
  }
}
