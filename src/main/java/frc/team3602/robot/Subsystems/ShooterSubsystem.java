/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.Subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.team3602.robot.Constants.shooterConstants.*;

import java.util.function.DoubleSupplier;

// import monologue.Logged;
// import monologue.Annotations.Log;

public class ShooterSubsystem extends SubsystemBase {
  //@Log
  private String defaultCommandName;

  // Motor controllers
 // @Log
  public double topOut;

 // @Log
  private double bottomOut;

 // @Log
  private double topVolts, bottomVolts;

  public final CANSparkMax topShooterMotor = new CANSparkMax(kTopShooterMotorId, MotorType.kBrushless);
  public final CANSparkMax bottomShooterMotor = new CANSparkMax(kBottomShooterMotorId, MotorType.kBrushless);

  // Encoders
  private final RelativeEncoder topShooterEncoder = topShooterMotor.getEncoder();
  private final RelativeEncoder bottomShooterEncoder = bottomShooterMotor.getEncoder();

  // Controls
//  @Log
  public boolean isAtVelocity;
//  @Log
  public boolean isAtSpeed;

//  @Log
  public double topVelocityRPM = 0, bottomVelocityRPM = 0; // 2300 trap_top: 1800, trap_bottom: 3000

 // @Log
  public double topSpeed = 0, bottomSpeed = 0;

  // private double __kP, __kI, __kD;

  private final SparkPIDController topController = topShooterMotor.getPIDController();
  private final SparkPIDController bottomController = bottomShooterMotor.getPIDController();

  public ShooterSubsystem() {
    SmartDashboard.putNumber("Shooter Top RPM", topVelocityRPM);
    SmartDashboard.putNumber("Shooter Bottom RPM", bottomVelocityRPM);

    // SmartDashboard.putNumber("Shooter kP", __kP);
    // SmartDashboard.putNumber("Shooter kI", __kI);
    // SmartDashboard.putNumber("Shooter kD", __kD);

    configShooterSubsys();
  }

 // @Log
  public double getTopEncoder() {
    return topShooterEncoder.getVelocity();
  }

 // @Log
  public double getBottomEncoder() {
    return bottomShooterEncoder.getVelocity();
  }

 // @Log
  public boolean atVelocity() {
    var topTarget = topVelocityRPM;
    var bottomTarget = bottomVelocityRPM;

    var tolerance = 100;

    boolean topOk = MathUtil.isNear(topTarget, getTopEncoder(), tolerance);
    boolean bottomOk = MathUtil.isNear(bottomTarget, getBottomEncoder(),
        tolerance);

    return topOk && bottomOk;
  }

//  @Log
  public boolean atSpeed() {
    var topTarget = topSpeed;
    var bottomTarget = bottomSpeed;

    var tolerance = 0.09;

    boolean topOk = MathUtil.isNear(topTarget, topShooterMotor.getAppliedOutput(), tolerance);
    boolean bottomOk = MathUtil.isNear(bottomTarget, bottomShooterMotor.getAppliedOutput(),
        tolerance);

    return topOk && bottomOk;
  }

  // public Command atVelocity(BooleanSupplier isFinishedSup) {
  // return new FunctionalCommand(
  // () -> {

  // },
  // () -> {
  // var topTarget = topVelocityRPM;
  // var bottomTarget = bottomVelocityRPM;

  // var tolerance = 600;

  // boolean topOk = MathUtil.isNear(topTarget, getTopEncoder(), tolerance);
  // boolean bottomOk = MathUtil.isNear(bottomTarget, getBottomEncoder(),
  // tolerance);

  // isAtVelocity = topOk && bottomOk;
  // }, (onEnd) -> {

  // }, isFinishedSup, this);
  // }

  // @Log
  // public boolean atVelocity() {
  // if ((getTopEncoder() >= 4500 && getBottomEncoder() >= 4500)) { // 3700
  // trap_top: 1300, trap_bottom: 2400
  // return true;
  // } else {
  // return false;
  // }
  // }

  public Command setTopRPM(DoubleSupplier velocityRPM) {
    return runOnce(() -> {
      topVelocityRPM = velocityRPM.getAsDouble();
    });
  }

  public Command setBottomRPM(DoubleSupplier velocityRPM) {
    return runOnce(() -> {
      bottomVelocityRPM = velocityRPM.getAsDouble();
    });
  }

  public Command setRPM(double topVelocityRPM, double bottomVelocityRPM) {
    return runOnce(() -> {
      this.topVelocityRPM = topVelocityRPM;
      this.bottomVelocityRPM = bottomVelocityRPM;
    });
  }

  public Command setSpeed(double topSpeed, double bottomSpeed) {
    return runOnce(() -> {
      this.topSpeed = topSpeed;
      this.bottomSpeed = bottomSpeed;
    });
  }

  public Command runShooterSpeed(double topSpeed, double bottomSpeed) {
    return run(() -> {
      topShooterMotor.set(topSpeed);
      bottomShooterMotor.set(bottomSpeed);
    });
  }

  // public Command runShooterSpeed() {
  // return run(() -> {
  // topShooterMotor.set(topSpeed);
  // bottomShooterMotor.set(bottomSpeed);
  // }).withName("Run Shooter Speed Default Command");
  // }

  public Command runShooterRPM(DoubleSupplier topVelocityRPM, DoubleSupplier bottomVelocityRPM) {
    return run(() -> {
      topController.setReference(topVelocityRPM.getAsDouble(), ControlType.kVelocity);
      bottomController.setReference(bottomVelocityRPM.getAsDouble(), ControlType.kVelocity);
    }).finallyDo(() -> {
      topShooterMotor.stopMotor();
      bottomShooterMotor.stopMotor();
    });
  }

  public Command runShooter() {
    return run(() -> {
      topController.setReference(topVelocityRPM, ControlType.kVelocity);
      bottomController.setReference(bottomVelocityRPM, ControlType.kVelocity);

      if (topVelocityRPM <= 100 && bottomVelocityRPM <= 100) {
        topVelocityRPM = 10;
        bottomVelocityRPM = 10;
      } else if (topVelocityRPM >= 100 && bottomVelocityRPM >= 100) {
        topController.setReference(topVelocityRPM, ControlType.kVelocity);
        bottomController.setReference(bottomVelocityRPM, ControlType.kVelocity);
      }
    }).withName("Run Shooter Default Command");
  }

  public Command stopShooter() {
    return runOnce(() -> {
      topVelocityRPM = 0;
      bottomVelocityRPM = 0;
    });
  }

  public Command stopMotorsCmd() {
    return runOnce(() -> {
      topShooterMotor.stopMotor();
      bottomShooterMotor.stopMotor();
    });
  }

  public void stopMotors() {
    topShooterMotor.stopMotor();
    bottomShooterMotor.stopMotor();
  }

  @Override
  public void periodic() {
    topOut = topShooterMotor.getAppliedOutput();
    bottomOut = bottomShooterMotor.getAppliedOutput();

    topVolts = (topShooterMotor.getAppliedOutput() / 12);
    bottomVolts = (bottomShooterMotor.getAppliedOutput() / 12);

    // defaultCommandName = this.getDefaultCommand().getName();

    isAtVelocity = atVelocity();
    isAtSpeed = atSpeed();

    // Get new tuning numbers from shuffleboard
    // var _kP = SmartDashboard.getNumber("Shooter kP", __kP);
    // var _kI = SmartDashboard.getNumber("Shooter kI", __kI);
    // var _kD = SmartDashboard.getNumber("Shooter kD", __kD);

    // Check if tuning numbers changed and update controller values
    // if (_kP != __kP) {
    // __kP = _kP;
    // topController.setP(__kP);
    // }
    // if (_kI != __kI) {
    // __kI = _kI;
    // topController.setI(__kI);
    // }
    // if (_kD != __kD) {
    // __kD = _kD;
    // topController.setD(__kD);
    // }

    // Get new velocity numbers from shuffleboard
    // var _topVelocityRPM = SmartDashboard.getNumber("Shooter Top RPM",
    // topVelocityRPM);
    // var _bottomVelocityRPM = SmartDashboard.getNumber("Shooter Bottom RPM",
    // bottomVelocityRPM);

    // Check if velocity numbers changed and update values
    // if ((_topVelocityRPM != topVelocityRPM)) {
    // topVelocityRPM = _topVelocityRPM;
    // }
    // if ((_bottomVelocityRPM != bottomVelocityRPM)) {
    // bottomVelocityRPM = _bottomVelocityRPM;
    // }
  }

  private void configShooterSubsys() {
    // Top shooter motor config
    topShooterMotor.restoreFactoryDefaults(true);
    topShooterMotor.setIdleMode(IdleMode.kCoast);
    topShooterMotor.setSmartCurrentLimit(kShooterCurrentLimit);
    topShooterMotor.enableVoltageCompensation(topShooterMotor.getBusVoltage());
    topShooterMotor.setOpenLoopRampRate(0.01);
    topShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);

    // Bottom shooter motor config
    bottomShooterMotor.restoreFactoryDefaults(true);
    bottomShooterMotor.setIdleMode(IdleMode.kCoast);
    bottomShooterMotor.setSmartCurrentLimit(kShooterCurrentLimit);
    bottomShooterMotor.enableVoltageCompensation(bottomShooterMotor.getBusVoltage());
    bottomShooterMotor.setOpenLoopRampRate(0.01);
    bottomShooterMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 500);

    // Encoder config
    topShooterEncoder.setMeasurementPeriod(10);
    topShooterEncoder.setAverageDepth(2);

    bottomShooterEncoder.setMeasurementPeriod(10);
    bottomShooterEncoder.setAverageDepth(2);

    // Controls config
    topController.setFeedbackDevice(topShooterEncoder);
    bottomController.setFeedbackDevice(bottomShooterEncoder);

    topController.setP(kP);
    topController.setI(kI);
    topController.setD(kD);

    bottomController.setP(kP);
    bottomController.setI(kI);
    bottomController.setD(kD);

    // topController.setSmartMotionAllowedClosedLoopError(5, 0);
    // bottomController.setSmartMotionAllowedClosedLoopError(5, 0);

    topShooterMotor.burnFlash();
    bottomShooterMotor.burnFlash();
  }
}
