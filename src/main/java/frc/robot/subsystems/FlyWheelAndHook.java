package frc.robot.subsystems;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.DigitalInput;


public class FlyWheelAndHook extends SubsystemBase{
    // Servo actuator = new Servo(1);
    //CODE STUB FOR THE 2024 Crescendo Season. 
    DigitalInput lSwitch;


    WPI_TalonSRX topfly = new WPI_TalonSRX(6);
    //WPI_TalonSRX topfly = new WPI_TalonSRX(5);
    CANSparkMax bottomfly = new CANSparkMax(10,MotorType.kBrushless);
    //RelativeEncoder hookencoder;
    CANSparkMax hook = new CANSparkMax(8,MotorType.kBrushless);
    SparkPIDController controller;// = hook.getPIDController();
    RelativeEncoder contenc;// = hook.getEncoder();
    public FlyWheelAndHook(){
        hook.restoreFactoryDefaults();
        controller = hook.getPIDController();
        contenc = hook.getEncoder();
        configureConstants();
        lSwitch = new DigitalInput(0);
    }

    public void configureConstants(){
        controller.setP(0.7);
        controller.setD(0);
        controller.setI(0.00001);
        controller.setIZone(0);
        controller.setFF(0);
        controller.setOutputRange(-.6,.6);
    }
    public void encoderLock(double pos){
        //-130 -> 0 are the positions, 0 is lowest, -130 is highest
        controller.setReference(pos,CANSparkMax.ControlType.kPosition);
    }
    public double getHookPos(){
        ////SmartDashboard.putNumber("Hook Posit: ", contenc.getPosition());
        return contenc.getPosition();
    }
    //SparkPIDController hookpid;
    // public void moveActuatorOn(){
    //     actuator.setAngle(180);
    //     System.out.println(actuator.getAngle());
    // }
    // public void moveActuatorOff(){
    //     actuator.setAngle(0);
    //     System.out.println(actuator.getAngle());
    // }
    // public void neutralActuator(){
    //     actuator.set(0);
    // }
    public void zeroHook(){
        contenc.setPosition(0);
    }
    public boolean getLimitSwitch(){
        return lSwitch.get();
        //return false;
    }

    public void FlyWheel(){
        //hookpid = hook.getPIDController();
    }
    public void moveHook(double speed){
        hook.set(speed);
    }
    public void moveFly(double speed){
        //SmartDashboard.putNumber("DFLY:",speed);
        bottomfly.set(speed);
        topfly.set(speed);
    }

    public void moveBottomFlyWheel(double speed){
        //SmartDashboard.putNumber("BFLY: ",speed);
        bottomfly.set(speed);
    }

    public void moveTopFlyWheel(double speed){
        //SmartDashboard.putNumber("TFLY: ",speed);
        topfly.set(speed);
    }





}
