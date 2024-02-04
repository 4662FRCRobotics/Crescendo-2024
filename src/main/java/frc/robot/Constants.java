// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public final class Constants {

    public static final class CANIDConstants {
        // CAN IDs
        // PDP and PNUMATICS are normaly 0 and 1 
        //drive gets 02 to 09
        //subsystem 1 gets 10 to 19
        //subsystem 2 gets 20 to 29
        //ect... 

        public static final int kFRONT_LEFT_DRIVING_CAN_ID = 2;
        public static final int kREAR_LEFT_DRIVING_CAN_ID = 4;
        public static final int kFRONT_RIGHT_DRIVING_CAN_ID = 6;
        public static final int kREAR_RIGHT_DRIVING_CAN_ID = 8;

        public static final int kFRONT_LEFT_TURNING_CAN_ID = 3;
        public static final int kREAR_LEFT_TURNING_CAN_ID = 5;
        public static final int kFRONT_RIGHT_TURNING_CAN_ID = 7;
        public static final int kREAR_RIGHT_TURNING_CAN_ID = 9;

        public static final int kSHOOTER_LEFT_MOTOR_ID = 10;
        public static final int kSHOOTER_RIGHT_MOTOR_ID = 11;

        public static final int kINTAKE_LIFT_MOTOR_ID = 20;
        public static final int kINTAKE_SPIN_MOTOR_ID = 21;
    }

    public static final class DriveConstants {
        public static final double kFREE_SPEED_RPM = 5676;
        // Driving Parameters - Note that these are not the maximum capable speeds of
        // the robot, rather the allowed maximum speeds
        public static final double kMAX_SPEED_METERS_PER_SECOND = 4.8;
        public static final double kMAX_ANGULAR_SPEED = 2 * Math.PI; // radians per second
        public static final double kMAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 4;
        public static final double kDIRECTION_SLEW_RATE = 1.2; // radians per second
        public static final double kMAGNITUDE_SLEW_RATE = 1.8; // percent per second (1 = 100%)
        public static final double KROTATIONAL_SLEW_RATE = 2.0; // percent per second (1 = 100%)


        public static final double kFRONT_LEFT_CHASSIS_ANGULAR_OFFSET = -Math.PI / 2;
        public static final double kFRONT_RIGHT_CHASSIS_ANGULAR_OFFSET = 0;
        public static final double kBACK_LEFT_CHASSIS_ANGULAR_OFFSET = Math.PI;
        public static final double kBACK_RIGHT_CHASSIS_ANGULAR_OFFSET = Math.PI / 2;

        public static final double kTRACK_WIDTH = Units.inchesToMeters(26.5);
        // Distance between centers of right and left wheels on robot
        public static final double kWHEEL_BASE = Units.inchesToMeters(26.5);

        // Distance between front and back wheels on robot
        public static final SwerveDriveKinematics kDRIVE_KINEMATICS = new SwerveDriveKinematics(
                new Translation2d(kWHEEL_BASE / 2, kTRACK_WIDTH / 2),
                new Translation2d(kWHEEL_BASE / 2, -kTRACK_WIDTH / 2),
                new Translation2d(-kWHEEL_BASE / 2, kTRACK_WIDTH / 2),
                new Translation2d(-kWHEEL_BASE / 2, -kTRACK_WIDTH / 2));
        // Chassis configuration

        public static final boolean kGYRO_REVERSED = false;
    }

    public static class SwerveConstants {
        public static final int kDRIVING_MOTOR_PINION_TEETH = 12;

        // Invert the turning encoder, since the output shaft rotates in the opposite
        // direction of
        // the steering motor in the MAXSwerve Module.
        public static final boolean kTURNING_ENCODER_INVERTED = true;

        // Calculations required for driving motor conversion factors and feed forward
        public static final double kDRIVING_MOTOR_FREE_SPEED_RPS = DriveConstants.kFREE_SPEED_RPM / 60;
        public static final double kWHEEL_DIAMETER_METERS = 0.0762;
        public static final double kWHEEL_CIRCUMFERENCE = kWHEEL_DIAMETER_METERS * Math.PI;
        // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
        // teeth on the bevel pinion
        public static final double kDRIVING_MOTOR_REDUCTION = (45.0 * 22) / (kDRIVING_MOTOR_PINION_TEETH * 15);
        public static final double kDRIVE_WHEEL_FREE_SPEED_RPS = (kDRIVING_MOTOR_FREE_SPEED_RPS * kWHEEL_CIRCUMFERENCE)
                / kDRIVING_MOTOR_REDUCTION;

        public static final double kDRIVING_ENCODER_POSITION_FACTOR = (kWHEEL_DIAMETER_METERS * Math.PI)
                / kDRIVING_MOTOR_REDUCTION; // meters
        public static final double kDRIVING_ENCODER_VELOCITY_FACTOR = ((kWHEEL_DIAMETER_METERS * Math.PI)
                / kDRIVING_MOTOR_REDUCTION) / 60.0; // meters per second

        public static final double kTURING_ENCODER_POSTION_FACTOR = (2 * Math.PI); // radians
        public static final double kTURNING_ENCODER_VELOCITY_FACTOR = (2 * Math.PI) / 60.0; // radians per second

        public static final double kTURNING_ENCODER_POSITION_PID_MIN_INPUT = 0; // radians
        public static final double kTURNING_ENCODER_POSITION_PID_MAX_INPUT = kTURING_ENCODER_POSTION_FACTOR; // radians

        public static final double kDRIVING_P = 0.04;
        public static final double kDRIVING_I = 0;
        public static final double kDRIVING_D = 0;
        public static final double kDRIVING_FF = 1 / kDRIVE_WHEEL_FREE_SPEED_RPS;
        public static final double kDRIVING_MIN_OUTPUT = -1;
        public static final double kDRVIVING_MAX_OUTPUT = 1;

        public static final double kTURNING_P = 1;
        public static final double kTURNING_I = 0;
        public static final double kTURNING_D = 0;
        public static final double kTURNING_FF = 0;
        public static final double kTURNING_MIN_OUTPUT = -1;
        public static final double kTURNING_MAX_OUTPUT = 1;

        public static final IdleMode kDRIVING_MOTOR_IDLE_MODE = IdleMode.kBrake;
        public static final IdleMode kTURNING_MOTOR_IDLE_MODE = IdleMode.kBrake;

        public static final int kDRIVING_MOTOR_CURRENT_LIMIT = 40; // amps
        public static final int kTURNING_MOTOR_CURRENT_LIMIT = 20; // amps

    }

    public static class ShooterConstants {
    public static final boolean kIS_INVERTED = true;
        
    }
    public static class OperatorConstants {

        public static final int kDRIVER_CONTROLLER_PORT = 0;
        public static final int kAUTONOMOUS_CONSOLE_PORT = 1;
    }

}
