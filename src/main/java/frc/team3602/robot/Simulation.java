package frc.team3602.robot;


import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;
import com.revrobotics.*;

import edu.wpi.first.math.proto.System;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismObject2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Simulation extends SubsystemBase{
/*our vars and such */
  //motors and sim motors
    //private static final CANSparkMax bruh = new CANSparkMax(1, MotorType.kBrushless);

    private static final TalonFX intakeMotor = new TalonFX(10);
    private static final TalonFXSimState simIntakeMotor = intakeMotor.getSimState();
    
    private static final TalonFX pivotMotor = new TalonFX(11);
    private static final TalonFXSimState simPivotMotor = pivotMotor.getSimState();
    
    private static final TalonFX shooterMotor = new TalonFX(12);
    private static final TalonFXSimState simShooterMotor = shooterMotor.getSimState();

    public double pivotPos;
    public double simPivotPos;

  
 //naming our sims
  //sim for intake
    private static final FlywheelSim intakeSim =
        new FlywheelSim(DCMotor.getNeo550(1), 5, 0.01);
  //sim for shooter
    private static final FlywheelSim shooterSim = 
        new FlywheelSim(DCMotor.getNEO(1), 1, 0.01);
  //sim for pivot
    private static final SingleJointedArmSim pivotSim = 
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
             48, 0.1, 0.6, /*0.14*/ 0.0, /*1.92*/ 3.0, true, 1.57);

  //2d visualizations
    //Creates the "background", is through smartdashboard
    private static final Mechanism2d mech = new Mechanism2d(1, 1);

    //places the objects in the background
    private static final MechanismRoot2d pivot = mech.getRoot("Pivot Root", 0.7, 0.3); 
    private static final MechanismLigament2d pivotViz = pivot.append( new MechanismLigament2d("pivot ligament", 0.5, 10.0, 5.0, new Color8Bit(Color.kHoneydew)));

    private static final MechanismRoot2d shooters = mech.getRoot("shooter Root", 0.7, 0.3);
    private static final MechanismLigament2d shooterViz = shooters.append(new MechanismLigament2d("Shooter rollers", 0.1, 0.0, 10.0, new Color8Bit(Color.kAquamarine)));
 
    private static final MechanismRoot2d intake = mech.getRoot("intake Root", 0.0, 0.0);
    private static final MechanismLigament2d intakeViz = intake.append(new MechanismLigament2d("intake rollers", 0.12, 0.0, 5.0, new Color8Bit(Color.kDarkOrchid)));

/* the good stuff, ie our commands and such */
    
    //our constructor
    public Simulation() {
            SmartDashboard.putData("Arm Viz", mech);

    }

    //stupid commands to put with button bindings for simulations
    public Command testShooter() {
        return runEnd(() -> {
            shooterMotor.set(0.8);
        },
        () -> {
            shooterMotor.set(0.0);
        } );
    }


    public Command testIntake() {
        return runEnd(() -> {
            intakeMotor.set(0.8);
        },
        () ->{
            intakeMotor.set(0.0);
    });
    }


    public Command testPivot() {
        return run(() -> {
            pivotMotor.set(0.8);
           
        });
    }

   public Command testPivotReverse() {
        return runOnce(() -> {
            pivotMotor.set(-0.8);
           
        });
    }

    


    @Override
    public void periodic() {
        intakeSim.setInput(simIntakeMotor.getMotorVoltage());
        intakeSim.update(TimedRobot.kDefaultPeriod);

        shooterSim.setInput(simShooterMotor.getMotorVoltage());
        shooterSim.update(TimedRobot.kDefaultPeriod);

        pivotSim.setInput(simPivotMotor.getMotorVoltage());
        pivotSim.update(TimedRobot.kDefaultPeriod);

        simPivotMotor.setRawRotorPosition((pivotSim.getAngleRads() - Math.PI/2)* 2.0 * Math.PI * 48); // 48 is the motor reduction ratio. math.pi/2 is the starting angle(90 degrees)
        simPivotMotor.setRotorVelocity(pivotSim.getVelocityRadPerSec() * 48 / (2.0 * Math.PI));

        double rotationsPerSecond = intakeSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        simIntakeMotor.setRotorVelocity(rotationsPerSecond);
        simIntakeMotor.addRotorPosition(rotationsPerSecond * 0.02); //0.02 is the default loop period

      //Updates the visual based on sim
        pivotViz.setAngle(Math.toDegrees(pivotSim.getAngleRads()));

        intake.setPosition((0.7 + 0.5 * Math.cos(pivotSim.getAngleRads())), (0.3 + 0.5 * Math.sin(pivotSim.getAngleRads())));
        intakeViz.setAngle(intakeViz.getAngle() + Math.toDegrees(intakeSim.getAngularVelocityRPM()) * 0.02 * 0.03);
    
        shooterViz.setAngle(shooterViz.getAngle() + Math.toDegrees(shooterSim.getAngularVelocityRPM()) * 0.02 * 0.03);

      //updates the pivot position
        pivotSim.getAngleRads();
        simPivotPos = pivotSim.getAngleRads();
    
    }

}