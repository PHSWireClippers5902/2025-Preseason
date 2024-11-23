package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwervePIDConstants;

public class SwerveModule extends SubsystemBase{
    public static final double kWheelRadius = 0.0508; //convert LATER, in m
    public static final int kEncoderResolution = 4096;
    //change later
    public static final double kGearReduction = 10.71;
    public static final double encoderToRadians = (2*Math.PI)/(kEncoderResolution);
    public static final double kMaxSpeed = 3.0; // 3 meters per second should be fine to work with 
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second

    CANSparkMax powerController;
    WPI_TalonSRX steeringController;
    RelativeEncoder powerEncoder;
    SparkPIDController powerPIDController;


    public SwerveModule(int powerID, int steeringID){
        powerController = new CANSparkMax(powerID,MotorType.kBrushless);
        powerEncoder = powerController.getEncoder();
        powerPIDController = powerController.getPIDController();
        powerEncoder.setPosition(0);
        // public static final double kDrivingP = 0.2;
        // public static final double kDrivingI = 0; 
        // public static final double kDrivingD = 0.002; 
        powerPIDController.setP(0.2);
        powerPIDController.setI(0);
        powerPIDController.setD(0.002);
        
        powerEncoder.setPositionConversionFactor(2*Math.PI*kWheelRadius/(42*kGearReduction));
        powerEncoder.setVelocityConversionFactor(2*Math.PI*kWheelRadius/(42*kGearReduction*60));

        // powerPIDController.setIZone();
        // powerPIDController.setFF()
        

        steeringController = new WPI_TalonSRX(steeringID);
        steeringController.configFactoryDefault();
        //sets the configured sensor to various predetermined constants
        steeringController.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);
        //ensure sensor is positive when output is positive
        steeringController.setSensorPhase(SwervePIDConstants.kSensorPhase);
        //sets the steeringController to inverted if the motor tells it to do so.
        steeringController.setInverted(SwervePIDConstants.kMotorInvert);
        //set peak and nominal outputs
        steeringController.configNominalOutputForward(0,SwervePIDConstants.kTimeoutMs);
        steeringController.configNominalOutputReverse(0,SwervePIDConstants.kTimeoutMs);
        steeringController.configPeakOutputForward(1,SwervePIDConstants.kTimeoutMs);
        steeringController.configPeakOutputReverse(-1,SwervePIDConstants.kTimeoutMs);
        //configures position closed loop in slot0
        steeringController.config_kF(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kF,SwervePIDConstants.kTimeoutMs);
        steeringController.config_kP(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kP,SwervePIDConstants.kTimeoutMs);
        steeringController.config_kI(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kI,SwervePIDConstants.kTimeoutMs);
        steeringController.config_kD(SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kGains.kD,SwervePIDConstants.kTimeoutMs);
        // steeringController.setDistancePerPulse();
        steeringController.configSelectedFeedbackCoefficient(2*Math.PI/kEncoderResolution,0,0);
        //sets the relative sensor to match absolute
        steeringController.setSelectedSensorPosition(0,SwervePIDConstants.kPIDLoopIdx,SwervePIDConstants.kTimeoutMs);


        
    }
    public SwerveModulePosition getPosition(){
        double distanceMeters = powerEncoder.getPosition();
        double angleRadians = steeringController.getSelectedSensorPosition() * encoderToRadians;
        return new SwerveModulePosition(
            distanceMeters,
            new Rotation2d(angleRadians)
        );
    }
    public void setDesiredState(SwerveModuleState desiredState){
        double currentAngleRadians = steeringController.getSelectedSensorPosition() * encoderToRadians;
        Rotation2d currentRotation = new Rotation2d(currentAngleRadians);
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState,currentRotation);
        double angleError = optimizedState.angle.minus(currentRotation).getRadians();
        double scaledSpeed = optimizedState.speedMetersPerSecond * Math.cos(angleError);

        double driveOutput = Math.max(-1, Math.min(1, scaledSpeed / kMaxSpeed)); // Clamp between -1 and 1
    
        powerController.set(driveOutput);
        
        double targetAngleRadians = optimizedState.angle.getRadians();
        double targetEncoderPosition = targetAngleRadians / encoderToRadians;
        steeringController.set(ControlMode.Position,targetEncoderPosition);

    }
    public SwerveModuleState getState(){
        double speedMPS = powerEncoder.getVelocity();
        double angleRadians = steeringController.getSelectedSensorPosition() * encoderToRadians;
        return new SwerveModuleState(speedMPS,new Rotation2d(angleRadians));

    }

}
