package frc.robot;

public final class Constants{
    public static final class SwervePIDConstants{
        public static final int kSlotIdx = 0;

        public static final int kPIDLoopIdx = 0;

        public static final int kTimeoutMs = 0;

        public static boolean kSensorPhase = true;

        public static boolean kMotorInvert = false;

        public static final Gains kGains = new Gains(0.8, 0, 0.5, 0, 0, 1.0);


    }
    public static final class SwerveMotorConstants{
        public static final int powerFLID = 1;
        public static final int controlFLID = 1;

        public static final int powerFRID = 2;
        public static final int controlFRID = 2;

        public static final int powerBLID = 3;
        public static final int controlBLID = 3;

        public static final int powerBRID = 4;
        public static final int controlBRID = 4;

    }
    
}