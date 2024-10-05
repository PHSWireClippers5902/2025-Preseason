package frc.robot.subsystems;

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
    
    
    public WPI_TalonSRX controlOne;
    public double controlOnePosition;
    public CANSparkMax powerOne;
    public SparkPIDController powerOneController;// = hook.getPIDController();
    public RelativeEncoder powerOneRelativeEncoder;// = hook.getEncoder();

    public SwerveSystem(){
        createMotorObjects();
        normalConfigurations();
    }

    public void createMotorObjects(){
        controlOne = new WPI_TalonSRX(SwerveMotorConstants.controlOneID);
        powerOne = new CANSparkMax(SwerveMotorConstants.powerOneID,MotorType.kBrushless);
    }
    //theory
    public void moveControlOne(double theta){
        double targetPosition = 4096 * theta / 360;
        controlOne.set(ControlMode.Position,targetPosition);
    }

    public void movePowerOne(double speed){
        powerOne.set(speed);
    }
    





    public void normalConfigurations(){
        //resets to default to avoid unexpected error
        controlOne.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        controlOne.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        controlOne.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the controlOne to inverted if the motor tells it to do so.
        controlOne.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        controlOne.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        controlOne.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        controlOne.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        controlOne.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        controlOne.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        controlOne.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        controlOne.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        controlOne.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);
        //get 360 deg position of magencoders absolute position
        int absolutePosition = controlOne.getSensorCollection().getPulseWidthPosition();
        //apparantly helps out with mask out overflows
        absolutePosition &=0xFFF;
        if (SwervePIDConstants.kSensorPhase) {absolutePosition*=-1;}
        if (SwervePIDConstants.kMotorInvert) {absolutePosition*=-1;}

        //sets the relative sensor to match absolute
        controlOne.setSelectedSensorPosition(absolutePosition,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
    }

    public double getControlOnePosition(){
        return controlOne.getSelectedSensorPosition();
    }
    public double getControlOneRotations(){
        return controlOne.getSelectedSensorPosition() / 4096;
    }

    
}
