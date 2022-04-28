// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Ports {

        public static final int FRONT_RIGHT_MOTOR_PORT = 1, FRONT_LEFT_MOTOR_PORT = 0,
                LEFT_ENCODER_PORT_A = 4, LEFT_ENCODER_PORT_B = 5,
                RIGHT_ENCODER_PORT_A = 6, RIGHT_ENCODER_PORT_B = 7;
    }

    public static class Controller {

        public static final int CONTROLLER_0 = 0,
                AXIS_LIN = 1, AXIS_ROT = 4,
                LEFT_BUMPER = 5, RIGHT_BUMPER = 6,
                A_BUTTON = 1, B_BUTTON = 2, X_BUTTON = 3, Y_BUTTON = 4,
                UP_ARROW = 5, RIGHT_ARROW = 6, DOWN_ARROW = 7, LEFT_ARROW = 8;
    }

    public static class Drive {

        public static final double DRIVE_MODIFIER = 1.0,
                TURN_MODIFIER = -1.0,

                AUTO_DISTANCE_KP = 1.25, AUTO_DISTANCE_KI = 0, AUTO_DISTANCE_KD = 0,
                AUTO_DISTANCE_MAX_V = 0.5, AUTO_DISTANCE_MAX_A = 0.5,

                AUTO_ROTATE_KP = 0.005, AUTO_ROTATE_KI = 0.002, AUTO_ROTATE_KD = 0,
                AUTO_ROTATE_MAX_V = 360, AUTO_ROTATE_MAX_A = 180,

                WHEEL_VELOCITY_KP = 1, WHEEL_VELOCITY_KI = 0, WHEEL_VELOCITY_KD = 0,
                
                RAMSETE_B = 2.0, RAMSETE_ZETA = 0.7,

                PATH_FOLLOW_MAX_M_PER_S = 1, PATH_FOLLOW_MAX_M_PER_SEC_SQUARED = 1,

                DRIVE_ENCODER_CPR = 1440,
                DRIVE_WHEEL_DIA_CM = 6.985,
                DRIVE_GEARBOX_RATIO = 1,
                DRIVE_METERS_PER_COUNT = 
                ((DRIVE_WHEEL_DIA_CM/100)*Math.PI)/(DRIVE_ENCODER_CPR*DRIVE_GEARBOX_RATIO),
                DRIVE_TRACK_WIDTH_CM = 14.1;
    }
}
