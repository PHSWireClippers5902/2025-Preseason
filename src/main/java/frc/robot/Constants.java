package frc.robot;

public final class Constants{
    public static final class SwervePIDConstants{
        public static final int kSlotIdx = 0;

        public static final int kPIDLoopIdx = 0;

        public static final int kTimeoutMs = 0;

        public static boolean kSensorPhase = true;

        public static boolean kMotorInvert = false;

        public static final Gains kGains = new Gains(0.2, 0.0, 1.0, 0, 0, 1.0);


    }
    public static final class SwerveMotorConstants{
        public static final int powerOneID = 7;
        public static final int controlOneID = 6;

        public static final int powerTwoID = 0;
        public static final int controlTwoID = 0;

        public static final int powerThreeID = 0;
        public static final int controlThreeID = 0;

        public static final int powerFourID = 0;
        public static final int controlFourID = 0;

    }
}