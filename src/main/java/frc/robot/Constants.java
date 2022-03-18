package frc.robot;

public final class Constants {
    public static final class DriveConstants {
        public static final int DRIVE_CONTROLLER = 0; // defined on port 0
        public static final int BUTTON_CONTROLLER = 1;
        public static final double SPEED = 0.2; // defined on port 0
    }

    public static final class ButtonConstants {
        // INTAKE
        public static final int INTAKE_ABSORB = 3;
        public static final int INTAKE_UNABSORB = 2;

        // ARM
        public static final int ARM_UP = 4;
        public static final int ARM_DOWN = 1;
    }
}