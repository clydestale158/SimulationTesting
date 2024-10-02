package frc.team3602.robot.Subsystems;


import java.lang.reflect.Array;

import javax.print.attribute.PrintJobAttribute;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.fasterxml.jackson.databind.type.ArrayType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.jni.CANSparkMaxJNI;
import com.revrobotics.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.proto.Pose3dProto;
import edu.wpi.first.math.geometry.struct.Pose3dStruct;
import edu.wpi.first.math.proto.System;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.Publisher;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
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

   // private static final TalonFX intakeMotor = new TalonFX(10);
   // $ private static final TalonFXSimState simIntakeMotor = intakeMotor.getSimState();
   //private static final TalonFXSimState simIntakeMotor ; 

    // private static final TalonFX pivotMotor = new TalonFX(11);
    // private static final TalonFXSimState simPivotMotor = pivotMotor.getSimState();
    
    // private static final TalonFX shooterMotor = new TalonFX(12);
    // private static final TalonFXSimState simShooterMotor = shooterMotor.getSimState();


    
private  _PivotSubsystem pivotSubsys;
private  ShooterSubsystem shooterSubsys;
private  IntakeSubsystem intakeSubsys;

    public double pivotPos;
    public double simPivotPos;

  
 //naming our sims
  //sim for intake
    private static final FlywheelSim intakeSim =
        new FlywheelSim(DCMotor.getNeo550(1), 5, 0.01);
  //sim for shooter
    private static final FlywheelSim shooterSim = 
        new FlywheelSim(DCMotor.getNEO(1), 1, 0.01);
        //sim for 2nd shooter
    private static final FlywheelSim bottomShooterSim = 
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

    private static final MechanismRoot2d shooter = mech.getRoot("shooter Root", 0.7, 0.3);
    private static final MechanismLigament2d shooterViz = shooter.append(new MechanismLigament2d("Shooter rollers", 0.1, 0.0, 10.0, new Color8Bit(Color.kAquamarine)));
    
    private static final MechanismRoot2d bottomShooter = mech.getRoot("shooter Root", 0.5, 0.3);
    private static final MechanismLigament2d bottomShooterViz = bottomShooter.append(new MechanismLigament2d("Bottom shooter rollers", 0.1, 0.0, 10.0, new Color8Bit(Color.kBlue)));
 
    private static final MechanismRoot2d intake = mech.getRoot("intake Root", 0.0, 0.0);
    private static final MechanismLigament2d intakeViz = intake.append(new MechanismLigament2d("intake rollers", 0.12, 0.0, 5.0, new Color8Bit(Color.kDarkOrchid)));



/* the good stuff, ie our commands and such */
    //our constructor
    public Simulation(IntakeSubsystem intakeSubsys, _PivotSubsystem pivotSubsys, ShooterSubsystem shooterSubsys) {
      this.intakeSubsys = intakeSubsys;
      this.pivotSubsys = pivotSubsys;
      this.shooterSubsys = shooterSubsys;
            SmartDashboard.putData("Arm Viz", mech);
         //  SmartDashboard.putNumberArray("POSE",);

    }

//     //stupid commands to put with button bindings for simulations
//     public Command testShooter() {
//         return runEnd(() -> {
//             shooterMotor.set(0.8);
//         },
//         () -> {
//             shooterMotor.set(0.0);
//         } );
//     }


//     public Command testIntake() {
//         return runEnd(() -> {
//             intakeMotor.set(0.8);
//         },
//         () ->{
//             intakeMotor.set(0.0);
//     });
//     }


//     public Command testPivot() {
//         return run(() -> {
//             pivotMotor.set(0.8);
           
//         });
//     }

//    public Command testPivotReverse() {
//         return runOnce(() -> {
//             pivotMotor.set(-0.8);
           
//         });
//     }

    
//NEW STUFF, SCARY AHHH
//Aka, 3d stuff

// x, y, z, w_rot, x_rot, y_rot, z_rot,
 // x, y, z, w_rot, x_rot, y_rot, z_rot,
//{0,0,0, 1, 0, 0, 0};



public Rotation3d rotation;

public Pose3d poseBruh;
//Pose3d poseB = new Pose3d();

    StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
    .getStructTopic("MyPose", Pose3d.struct).publish();

// = new Rotation3d(0, 0, 0);
// WPILib
// StructPublisher<Pose3d> publisher = NetworkTableInstance.getDefault()
//     .getStructTopic("MyPose", Pose3d.struct).publish();
// StructArrayPublisher<Pose3d> arrayPublisher = NetworkTableInstance.getDefault()
//     .getStructArrayTopic("MyPoseArray", Pose3d.struct).publish();

  

    @Override
    public void periodic() {
        // intakeSim.setInput(intakeMotor.getMotorVoltage().getValueAsDouble()); // $$
        intakeSim.setInput(intakeSubsys.intakeMotor.getAppliedOutput() / 12);
        intakeSim.update(TimedRobot.kDefaultPeriod);

        // shooterSim.setInput(simShooterMotor.getMotorVoltage());
        shooterSim.setInput(shooterSubsys.topShooterMotor.getAppliedOutput() / 12);
        shooterSim.update(TimedRobot.kDefaultPeriod);

         bottomShooterSim.setInput(shooterSubsys.bottomShooterMotor.getAppliedOutput() / 12 );
         bottomShooterSim.update(TimedRobot.kDefaultPeriod);


        // pivotSim.setInput(simPivotMotor.getMotorVoltage());
        pivotSim.setInput(pivotSubsys.pivotMotor.getAppliedOutput() /12 );
        pivotSim.update(TimedRobot.kDefaultPeriod);

        // simPivotMotor.setRawRotorPosition((pivotSim.getAngleRads() - Math.PI/2)* 2.0 * Math.PI * 48); // 48 is the motor reduction ratio. math.pi/2 is the starting angle(90 degrees)
        // simPivotMotor.setRotorVelocity(pivotSim.getVelocityRadPerSec() * 48 / (2.0 * Math.PI));

        double rotationsPerSecond = intakeSim.getAngularVelocityRadPerSec() / (2.0 * Math.PI);
        // simIntakeMotor.setRotorVelocity(rotationsPerSecond);
        // simIntakeMotor.addRotorPosition(rotationsPerSecond * 0.02); //0.02 is the default loop period

      //Updates the visual based on sim
        pivotViz.setAngle(Math.toDegrees(pivotSim.getAngleRads()));

        intake.setPosition((0.7 + 0.5 * Math.cos(pivotSim.getAngleRads())), (0.3 + 0.5 * Math.sin(pivotSim.getAngleRads())));
        intakeViz.setAngle(intakeViz.getAngle() + Math.toDegrees(intakeSim.getAngularVelocityRPM()) * 0.02 * 0.03);
    
        shooterViz.setAngle(shooterViz.getAngle() + Math.toDegrees(shooterSim.getAngularVelocityRPM()) * 0.02 * 0.03);

        bottomShooter.setPosition((0.5 - (0.2 * (0.7 + 0.5 * Math.cos(pivotSim.getAngleRads())))), (0.3-(0.2 * (0.3 + 0.5 * Math.sin(pivotSim.getAngleRads())))));
        bottomShooterViz.setAngle(shooterViz.getAngle() + Math.toDegrees(shooterSim.getAngularVelocityRPM()) * 0.02 * 0.03);


      //updates the pivot position
        pivotSim.getAngleRads();
        simPivotPos = pivotSim.getAngleRads();

    //3D Crap
    
        
        rotation = new Rotation3d(0,( -simPivotPos + 90), 0);
        poseBruh  = new Pose3d(0, 0, 0, rotation);
        publisher.set(poseBruh);
       // arrayPublisher.set(new Pose3d[] {poseA, poseB});
    
    }

}
 