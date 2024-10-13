/*
 * Copyright (C) 2024, FRC Team 3602. All rights reserved. This work
 * is licensed under the terms of the MIT license which can be found
 * in the root directory of this project.
 */

package frc.team3602.robot.Subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

 import static frc.team3602.robot.Constants.pivotConstants.*;

// import monologue.Logged;
// import monologue.Annotations.Log;

public class PivotSubsystem extends SubsystemBase {
    //subsystems
  private IntakeSubsystem intakeSubsys;
  private ShooterSubsystem shooterSubsys;

  // Motor controllers
  public final CANSparkMax pivotMotor = new CANSparkMax(kPivotLeaderId, MotorType.kBrushless);
  private final CANSparkMax pivotFollower = new CANSparkMax(kPivotFollowerId, MotorType.kBrushless);
  private final SparkLimitSwitch lowLimitSwitch = pivotMotor.getReverseLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);
  private final SparkLimitSwitch highLimitSwitch = pivotMotor.getForwardLimitSwitch(SparkLimitSwitch.Type.kNormallyClosed);

  // Encoders
  private final DutyCycleEncoder pivotEncoder = new DutyCycleEncoder(2);

  // Controls
  private final PIDController controller = new PIDController(kP, kI, kD);
  private final ArmFeedforward feedforward = new ArmFeedforward(kS, kG, kV, kA);

  //2D MECH sim stuff
    private double simPivotEncoder; //"Encoder" 

    private static final SingleJointedArmSim pivotSim = 
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
             48, SingleJointedArmSim.estimateMOI(0.8, 6.3), 0.6, /*0.14*/ 0.0 , /*1.92*/ 3.0, /*true*/true, /*1.57*/ 0.698);
    private static final FlywheelSim intakeSim =
        new FlywheelSim(DCMotor.getNeo550(1), 5, 0.01);
    private static final FlywheelSim shooterSim = 
        new FlywheelSim(DCMotor.getNEO(1), 1, 0.05);
    private static final FlywheelSim bottomShooterSim = 
        new FlywheelSim(DCMotor.getNEO(1), 1, 0.05);

    private static final Mechanism2d mech = new Mechanism2d(1, 1);
    private static final MechanismRoot2d pivotRoot = mech.getRoot("Pivot Root", 0.7, 0.3); 
    private static final MechanismLigament2d pivotViz = pivotRoot.append( new MechanismLigament2d("pivot ligament", 0.5, 10.0, 5.0, new Color8Bit(Color.kHoneydew)));
    private static final MechanismRoot2d shooterRoot = mech.getRoot("shooter Root", 0.7, 0.3);
    private static final MechanismLigament2d shooterViz = shooterRoot.append(new MechanismLigament2d("Shooter rollers", 0.1, 0.0, 10.0, new Color8Bit(Color.kAquamarine)));
    private static final MechanismRoot2d bottomShooterRoot = mech.getRoot("bottom Shooter Root", 0.7, 0.3);
    private static final MechanismLigament2d bottomShooterViz = bottomShooterRoot.append(new MechanismLigament2d("bottom Shooter rollers", 0.1, 0.0, 10.0, new Color8Bit(Color.kCadetBlue)));
    private static final MechanismRoot2d intakeRoot = mech.getRoot("intake Root", 0.0, 0.0);
    private static final MechanismLigament2d intakeViz = intakeRoot.append(new MechanismLigament2d("intake rollers", 0.12, 0.0, 5.0, new Color8Bit(Color.kDarkOrchid)));

    public boolean simIsAtPosition;

    //3D pose sim stuff
    public Rotation3d rotation;
    public Pose3d pivotPose;

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("Pivot Pose", Pose3d.struct).publish();
  



//random doubles
 // @Log
  public boolean isAtPosition, lowLimit, highLimit;

  //@Log
  public double encoderValue;

 // @Log
  public double angle = 40; //84
 // @Log
  public double lerpAngle;
  public double absoluteOffset = 77;

 // @Log
  public double effort;

 // @Log
  public double ffEffort;

 // @Log
  public double pidEffort;
    //@Log
  public double motorOutput, motorOutputTwo;

  public double totalEffort;
  //private final Vision vision = new Vision();

 // public final InterpolatingDoubleTreeMap lerpTable = new InterpolatingDoubleTreeMap();


  public PivotSubsystem(IntakeSubsystem intakeSubsys, ShooterSubsystem shooterSubsys) {
    this.intakeSubsys = intakeSubsys;
    this.shooterSubsys = shooterSubsys;

    SmartDashboard.putData("Pivot Viz", mech);

    configPivotSubsys();
  }



  public double getDegrees() {
    return (pivotEncoder.getAbsolutePosition() * 360) - absoluteOffset;
  }

  public boolean atPosition() {
    var target = angle;
    var tolerance = 2;

    return ((MathUtil.isNear(target, getDegrees(), tolerance)));
  }

  private double getEffort() {
    var ffEffort = feedforward.calculate(Units.degreesToRadians(getDegrees()), 0);
    var pidEffort = controller.calculate(getDegrees(), angle);
   
    this.ffEffort = ffEffort;
    this.pidEffort = pidEffort;

    return ffEffort + pidEffort;
  }



  public Command runSetAngle(DoubleSupplier angleDegrees) {
    return run(() -> {
      angle = angleDegrees.getAsDouble();

      var effort = getEffort();
      this.effort = effort;

      pivotMotor.setVoltage(effort);
    });
  }

  public Command holdAngle() {
    return run(() -> {
      var effort = getEffort();
      this.effort = effort;      
      
      pivotMotor.setVoltage(effort);
    });
  }


  //   public double simGetDegrees() {
  //   return simPivotEncoder * 180 / Math.PI;
  // }



  // private double simGetEffort() {
  //   var ffEffort = feedforward.calculate(Units.degreesToRadians(simGetDegrees()), 0);
  //   var pidEffort = controller.calculate(simGetDegrees(), angle);
   
  //  this.ffEffort = ffEffort;
  //   this.pidEffort = pidEffort;

  // return ffEffort + pidEffort;
  // }



 

  public double simGetEffort(){
    return totalEffort = ((feedforward.calculate(simPivotEncoder, 0)) + (controller.calculate(Units.radiansToDegrees(simPivotEncoder), angle)));
  }

  public Command simSetAngle(double newAngle) {
    return runOnce(() ->{
      angle =  newAngle;
    });
  }




  public Command stopMotors() {
    return runOnce(() -> {
      pivotMotor.stopMotor();
      pivotFollower.stopMotor();
    });
  }




  @Override
  public void periodic() {
    SmartDashboard.putNumber("Angle", angle);
    SmartDashboard.putNumber("SimPivotEncoder", Units.radiansToDegrees(simPivotEncoder));
    SmartDashboard.putNumber("Ffeffort", ffEffort);
    SmartDashboard.putNumber("PidEffort", pidEffort);
    SmartDashboard.putNumber("Total Effort", totalEffort);

    totalEffort = simGetEffort();

    pivotMotor.setVoltage(totalEffort);

    motorOutput = pivotMotor.getAppliedOutput();
    motorOutputTwo = pivotFollower.getAppliedOutput();

    
    encoderValue = getDegrees();

    isAtPosition = atPosition();    

    //lerpAngle = lerpTable.get(Units.metersToFeet(vision.getTargetDistance()));

    lowLimit = lowLimitSwitch.isPressed();
    highLimit = highLimitSwitch.isPressed();


    //Sim stuff
    
    pivotSim.setInput(pivotMotor.getAppliedOutput() /12 );
    pivotSim.update(TimedRobot.kDefaultPeriod);

    intakeSim.setInput(intakeSubsys.intakeMotor.getAppliedOutput() / 12);
    intakeSim.update(TimedRobot.kDefaultPeriod);

    shooterSim.setInput(shooterSubsys.topShooterMotor.getAppliedOutput() / 12);
    shooterSim.update(TimedRobot.kDefaultPeriod);

    bottomShooterSim.setInput(shooterSubsys.bottomShooterMotor.getAppliedOutput() / 12);
    bottomShooterSim.update(TimedRobot.kDefaultPeriod);


    pivotViz.setAngle(Math.toDegrees(pivotSim.getAngleRads()));
    intakeViz.setAngle(intakeViz.getAngle() + Math.toDegrees(intakeSim.getAngularVelocityRPM()) * 0.02 * 0.03);
    shooterViz.setAngle(shooterViz.getAngle() + Math.toDegrees(shooterSim.getAngularVelocityRPM()) * 0.02 * 0.03);
    bottomShooterViz.setAngle(bottomShooterViz.getAngle() + Math.toDegrees(bottomShooterSim.getAngularVelocityRPM()) * 0.02 * 0.03);

    bottomShooterRoot.setPosition((0.7 + 0.1 * Math.cos(pivotSim.getAngleRads())), (0.3 + 0.1 * Math.sin(pivotSim.getAngleRads())));
    intakeRoot.setPosition((0.7 + 0.5 * Math.cos(pivotSim.getAngleRads())), (0.3 + 0.5 * Math.sin(pivotSim.getAngleRads())));

    pivotSim.getAngleRads();
    simPivotEncoder = pivotSim.getAngleRads();

    rotation = new Rotation3d(0,( -simPivotEncoder + 90), 0);
    pivotPose  = new Pose3d(0, 0, 0, rotation);
    publisher.set(pivotPose);
  }



  private void configPivotSubsys() {
    // Pivot motor config
    pivotMotor.setIdleMode(IdleMode.kBrake);
    pivotMotor.setInverted(false);
    pivotMotor.setSmartCurrentLimit(kPivotCurrentLimit);
    pivotMotor.enableVoltageCompensation(pivotMotor.getBusVoltage());

    // Pivot motor follower config
    pivotFollower.setIdleMode(IdleMode.kBrake);
    pivotFollower.follow(pivotMotor, true);
    pivotFollower.setSmartCurrentLimit(kPivotCurrentLimit);
    pivotFollower.enableVoltageCompensation(pivotFollower.getBusVoltage());
    controller.setTolerance(1);

    lowLimitSwitch.enableLimitSwitch(true);
    highLimitSwitch.enableLimitSwitch(true);

    pivotMotor.burnFlash();
    pivotFollower.burnFlash();

    // // Interpolation table config
    // lerpTable.put(4.6, 32.0); // 4.6 feet, 32 degrees
    // lerpTable.put(6.87, 42.0); // 7.65 feet, 42 degrees
    // lerpTable.put(8.33,44.0); 
    // lerpTable.put(9.91,48.0); 
    // lerpTable.put(10.95, 50.0); // 10.7 feet, 49.75 degrees
    // lerpTable.put(11.4,51.0); 
    // lerpTable.put(12.53,50.0 ); 
    // lerpTable.put(13.47, 51.0); // 13.79 feet, 53 degrees
    // lerpTable.put(15.24,51.0 ); 
    // lerpTable.put(16.0, 50.0); // 16.9 feet, 52.5 degrees

  }
}
