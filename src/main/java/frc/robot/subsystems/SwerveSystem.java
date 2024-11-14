package frc.robot.subsystems;

import javax.swing.text.Position;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.proto.Controller;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveMotorConstants;
import frc.robot.Constants.SwervePIDConstants;


public class SwerveSystem extends SubsystemBase {
    
    
    public WPI_TalonSRX controlFrontLeft;
    public double controlFrontLeftPosition;
    public CANSparkMax powerFrontLeft;
    public boolean reverseFrontLeft;
    public SparkPIDController powerFrontLeftController;// = hook.getPIDController();
    public RelativeEncoder powerFrontLeftRelativeEncoder;// = hook.getEncoder();

    public WPI_TalonSRX controlFrontRight;
    public double controlFrontRightPosition;
    public CANSparkMax powerFrontRight;
    public boolean reverseFrontRight;
    public SparkPIDController powerFrontRightController;// = hook.getPIDController();
    public RelativeEncoder powerFrontRightRelativeEncoder;// = hook.getEncoder();

    public WPI_TalonSRX controlBackLeft;
    public double controlBackLeftPosition;
    public CANSparkMax powerBackLeft;
    public boolean reverseBackLeft;
    public SparkPIDController powerBackLeftController;// = hook.getPIDController();
    public RelativeEncoder powerBackLeftRelativeEncoder;// = hook.getEncoder();

    public WPI_TalonSRX controlBackRight;
    public double controlBackRightPosition;
    public CANSparkMax powerBackRight;
    public boolean reverseBackRight;
    public SparkPIDController powerBackRightController;// = hook.getPIDController();
    public RelativeEncoder powerBackRightRelativeEncoder;// = hook.getEncoder();

    public double controlFrontLeftAngle;
    public double controlFrontRightAngle;
    public double controlBackLeftAngle;
    public double controlBackRightAngle;

    public SwerveSystem(){
        createMotorObjects();
        normalConfigurations();
    }

    public void createMotorObjects(){
        controlFrontLeft = new WPI_TalonSRX(SwerveMotorConstants.controlFLID);
        powerFrontLeft = new CANSparkMax(SwerveMotorConstants.powerFLID,MotorType.kBrushless);

        
        controlFrontRight = new WPI_TalonSRX(SwerveMotorConstants.controlFRID);
        powerFrontRight = new CANSparkMax(SwerveMotorConstants.powerFRID,MotorType.kBrushless);

        
        controlBackLeft = new WPI_TalonSRX(SwerveMotorConstants.controlBLID);
        powerBackLeft = new CANSparkMax(SwerveMotorConstants.powerBLID,MotorType.kBrushless);

        
        controlBackRight = new WPI_TalonSRX(SwerveMotorConstants.controlBRID);
        powerBackRight = new CANSparkMax(SwerveMotorConstants.powerBRID,MotorType.kBrushless);

        controlFrontLeftAngle = 0;
        controlFrontRightAngle = 0;
        controlBackLeftAngle = 0;
        controlBackRightAngle = 0;

    }



    public void moveControlFrontLeft(double alpha){
        // double theta = getControlFrontLeftPosition()*360/4096;
        // if (Math.abs(theta-alpha)>180){
        //     if (theta-alpha > 180){
        //        theta-=360;  
        //     }
        //     else {
        //        theta+=360;
        //     }
        // }
        // setFrontLeftPositionCode(theta);
        controlFrontLeft.set(ControlMode.Position,alpha*4096/360);
    }
    public void moveControlFrontRight(double alpha){
        // double theta = getControlFrontRightPosition()*360/4096;
        // if (Math.abs(theta-alpha)>180){
        //     if (theta-alpha > 180){
        //        theta-=360;  
        //     }
        //     else {
        //        theta+=360;
        //     }
        // }
        // setFrontRightPositionCode(theta);
        controlFrontRight.set(ControlMode.Position,alpha*4096/360);
    }
    public void moveControlBackLeft(double alpha){
        // double theta = getControlBackLeftPosition()*360/4096;
        // if (Math.abs(theta-alpha)>180){
        //     if (theta-alpha > 180){
        //        theta-=360;  
        //     }
        //     else {
        //        theta+=360;
        //     }
        // }
        // setBackLeftPositionCode(theta);
        controlBackLeft.set(ControlMode.Position,alpha*4096/360);
    }
    public void moveControlBackRight(double alpha){
        // double theta = getControlBackRightPosition()*360/4096;
        // if (Math.abs(theta-alpha)>180){
        //     if (theta-alpha > 180){
        //        theta-=360;  
        //     }
        //     else {
        //        theta+=360;
        //     }
        // }
        // setBackRightPositionCode(theta);
        controlBackRight.set(ControlMode.Position,alpha*4096/360);
    }








    //theory
    //frontleft
    public void setFrontLeftPositionCode(double theta){
        controlFrontLeft.setSelectedSensorPosition(theta*4096/360,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
    }
    public void setFrontRightPositionCode(double theta){
        controlFrontRight.setSelectedSensorPosition(theta*4096/360,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);

    }
    public void setBackLeftPositionCode(double theta){
        controlBackLeft.setSelectedSensorPosition(theta*4096/360,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);

    }
    public void setBackRightPositionCode(double theta){
        controlBackRight.setSelectedSensorPosition(theta*4096/360,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
    }
    public void movePowerFrontLeft(double speed){
        powerFrontLeft.set(speed);
    }
    //front right
    

    public void movePowerFrontRight(double speed){
        powerFrontRight.set(speed);
    }
    //back left
    

    public void movePowerBackLeft(double speed){
        powerBackLeft.set(speed);
    }
    //back right
    

    public void movePowerBackRight(double speed){
        powerBackRight.set(speed);
    }

    //all power
    public void moveAllPower(double speedFrontLeft, double speedFrontRight, double speedBackLeft, double speedBackRight){
        movePowerFrontLeft(speedFrontLeft);
        movePowerFrontRight(speedFrontRight);
        movePowerBackLeft(speedBackLeft);
        movePowerBackRight(speedBackRight);
    }
    //all control
    public void moveAllControl(){
        // moveControlFrontLeft(controlFrontLeftAngle);
        // moveControlFrontRight(controlFrontRightAngle);
        // moveControlBackLeft(controlBackLeftAngle);
        // moveControlBackRight(controlBackRightAngle);
    }

    

    
    //front left
    public void changeDirectionsFrontLeft(){
        reverseFrontLeft = !reverseFrontLeft;
        powerFrontLeft.setInverted(reverseFrontLeft);
    }
    
    public double getControlFrontLeftPosition(){
        return controlFrontLeft.getSelectedSensorPosition();
    }
    public double getControlFrontLeftRotations(){
        return controlFrontLeft.getSelectedSensorPosition() / 4096;
    }
    //front right
    public void changeDirectionsFrontRight(){
        reverseFrontRight = !reverseFrontRight;
        powerFrontRight.setInverted(reverseFrontRight);
    }
    
    public double getControlFrontRightPosition(){
        return controlFrontRight.getSelectedSensorPosition();
    }
    public double getControlFrontRightRotations(){
        return controlFrontRight.getSelectedSensorPosition() / 4096;
    }
    //back left
    public void changeDirectionsBackLeft(){
        reverseBackLeft = !reverseBackLeft;
        powerBackLeft.setInverted(reverseBackLeft);
    }
    
    public double getControlBackLeftPosition(){
        return controlBackLeft.getSelectedSensorPosition();
    }
    public double getControlBackLeftRotations(){
        return controlBackLeft.getSelectedSensorPosition() / 4096;
    }
    //back right
    public void changeDirectionsBackRight(){
        reverseBackRight = !reverseBackRight;
        powerBackRight.setInverted(reverseBackRight);
    }
    
    public double getControlBackRightPosition(){
        return controlBackRight.getSelectedSensorPosition();
    }
    public double getControlBackRightRotations(){
        return controlBackRight.getSelectedSensorPosition() / 4096;
    }

    public void zeroMotors(){
        // normalConfigurations();
        controlFrontLeft.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        controlFrontRight.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        controlBackLeft.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        controlBackRight.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);

    }
    
    
    public void changeAllAngles(double thetaFL, double thetaFR, double thetaBL, double thetaBR){
        controlFrontLeftAngle = thetaFL;
        controlFrontRightAngle = thetaFR;
        controlBackLeftAngle = thetaBL;
        controlBackRightAngle = thetaBR;
    }
    public void changeFrontLeftAngle(double theta){
        controlFrontLeftAngle = theta;
    }
    public void changeFrontRightAngle(double theta){
        controlFrontRightAngle = theta;
    }
    public void changeBackLeftAngle(double theta){
        controlBackLeftAngle = theta;
    }
    public void changeBackRightAngle(double theta){
        controlBackRightAngle = theta;
    }
    public void normalConfigurations(){
        //resets to default to avoid unexpected error
        controlFrontLeft.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        controlFrontLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        controlFrontLeft.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the controlFrontLeft to inverted if the motor tells it to do so.
        controlFrontLeft.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        controlFrontLeft.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        controlFrontLeft.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        controlFrontLeft.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        controlFrontLeft.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        controlFrontLeft.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        controlFrontLeft.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        controlFrontLeft.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        controlFrontLeft.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);
        //get 360 deg position of magencoders absolute position
        int absolutePosition = controlFrontLeft.getSensorCollection().getPulseWidthPosition();
        //apparantly helps out with mask out overflows
        // absolutePosition &=0xFFF;
        if (SwervePIDConstants.kSensorPhase) {absolutePosition*=-1;}
        if (SwervePIDConstants.kMotorInvert) {absolutePosition*=-1;}
        //sets the relative sensor to match absolute
        controlFrontLeft.setSelectedSensorPosition(absolutePosition,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);

        reverseFrontLeft = false;
        powerFrontLeft.setInverted(reverseFrontLeft);





        //resets to default to avoid unexpected error
        controlFrontRight.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        controlFrontRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        controlFrontRight.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the controlFrontRight to inverted if the motor tells it to do so.
        controlFrontRight.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        controlFrontRight.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        controlFrontRight.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        controlFrontRight.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        controlFrontRight.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        controlFrontRight.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        controlFrontRight.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        controlFrontRight.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        controlFrontRight.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);
        //get 360 deg position of magencoders absolute position
        absolutePosition = controlFrontRight.getSensorCollection().getPulseWidthPosition();
        //apparantly helps out with mask out overflows
        // absolutePosition &=0xFFF;
        if (SwervePIDConstants.kSensorPhase) {absolutePosition*=-1;}
        if (SwervePIDConstants.kMotorInvert) {absolutePosition*=-1;}
        //sets the relative sensor to match absolute
        controlFrontRight.setSelectedSensorPosition(absolutePosition,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);

        reverseFrontRight = false;
        powerFrontRight.setInverted(reverseFrontRight);




        //resets to default to avoid unexpected error
        controlBackLeft.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        controlBackLeft.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        controlBackLeft.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the controlBackLeft to inverted if the motor tells it to do so.
        controlBackLeft.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        controlBackLeft.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        controlBackLeft.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        controlBackLeft.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        controlBackLeft.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        controlBackLeft.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        controlBackLeft.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        controlBackLeft.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        controlBackLeft.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);
        //get 360 deg position of magencoders absolute position
        absolutePosition = controlBackLeft.getSensorCollection().getPulseWidthPosition();
        //apparantly helps out with mask out overflows
        // absolutePosition &=0xFFF;
        if (SwervePIDConstants.kSensorPhase) {absolutePosition*=-1;}
        if (SwervePIDConstants.kMotorInvert) {absolutePosition*=-1;}
        //sets the relative sensor to match absolute
        controlBackLeft.setSelectedSensorPosition(absolutePosition,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);

        reverseBackLeft = false;
        powerBackLeft.setInverted(reverseBackLeft);





        //resets to default to avoid unexpected error
        controlBackRight.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        controlBackRight.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        controlBackRight.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the controlBackRight to inverted if the motor tells it to do so.
        controlBackRight.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        controlBackRight.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        controlBackRight.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        controlBackRight.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        controlBackRight.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        controlBackRight.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        controlBackRight.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        controlBackRight.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        controlBackRight.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);
        //get 360 deg position of magencoders absolute position
        absolutePosition = controlBackRight.getSensorCollection().getPulseWidthPosition();
        //apparantly helps out with mask out overflows
        // absolutePosition &=0xFFF;
        if (SwervePIDConstants.kSensorPhase) {absolutePosition*=-1;}
        if (SwervePIDConstants.kMotorInvert) {absolutePosition*=-1;}
        //sets the relative sensor to match absolute
        controlBackRight.setSelectedSensorPosition(absolutePosition,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);

        reverseBackRight = false;
        powerBackRight.setInverted(reverseBackRight);
    }
    public double angleGetter(double x, double y){
        //gets angle based on very complicated math (really just applied trig)
        if (y >= 0){
            if (x == 1){
                return Math.asin(y)*180/Math.PI;
            }
            else if (x == -1){
                return 180 - Math.asin(y)*180/Math.PI;
            }
            else if (y == 1){
                if (x >= 0){
                    return Math.acos(x)*180/Math.PI;
                }
                else {
                    return 180-Math.acos(x)*180/Math.PI;
                }
            }
            else {
                if (x >= 0){
                    return Math.atan(y/x)*180/Math.PI;
                }
                else {
                    return 180-Math.atan(-y/x)*Math.PI;
                }
            }
        }
        else {
            if (x == 1){
                return 360 - Math.asin(-y)*180/Math.PI;
            }
            else if (y == -1){
                if (x >= 0){
                    return 360-Math.acos(x)*180/Math.PI;
                }
                else {
                    return 180-Math.acos(x)*180/Math.PI;
                }
            }
            else {
                if (x >= 0){
                    return 360-Math.atan(y/x)*180/Math.PI;
                }
                else {
                    return 180+Math.atan(y/x)*180/Math.PI;
                }
            }
        }
    }
}
