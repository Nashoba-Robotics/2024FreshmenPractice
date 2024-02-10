// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static final double TAU = Math.PI * 2;

  public static class Drive {
    public static final String CANBUS = "drivet";

    public static final double MAX_VELOCITY = 3.70;
    public static final double MAX_ROTATION_VELOCITY = 9.30;

    public static final double WIDTH = Units.inchesToMeters(21.75);
      public static final double LENGTH = Units.inchesToMeters(21.75);

      public static final Slot0Configs steerGains0 = new Slot0Configs()
          .withKP(20).withKI(0).withKD(0.0)
            // .withKP(0).withKI(0).withKD(0)
          .withKS(0.25).withKV(2.615).withKA(0);

      public static final Slot0Configs steerGains1 = new Slot0Configs()
          .withKP(20).withKI(0).withKD(0.0)
      // .withKP(0).withKI(0).withKD(0)
          .withKS(0.27).withKV(2.590).withKA(0);
        
      public static final Slot0Configs steerGains2 = new Slot0Configs()
          .withKP(20).withKI(0).withKD(0.0)
      // .withKP(0).withKI(0).withKD(0)
          .withKS(0.28).withKV(2.600).withKA(0);

      public static final Slot0Configs steerGains3 = new Slot0Configs()
          .withKP(20).withKI(0).withKD(0.0)
      //     .withKP(0).withKI(0).withKD(0)
          .withKS(0.34).withKV(2.681).withKA(0); 



      private static final Slot0Configs driveGains = new Slot0Configs()
          .withKP(0.22).withKI(0).withKD(0)
      // .withKP(0).withKI(0).withKD(0)
          .withKS(0).withKV(0.1165).withKA(0);

      private static final ClosedLoopOutputType steerClosedLoopOutput = ClosedLoopOutputType.Voltage;
      private static final ClosedLoopOutputType driveClosedLoopOutput = ClosedLoopOutputType.Voltage;

      private static final double kSlipCurrentA = 60;

      public static final double kSpeedAt12VoltsMps = MAX_VELOCITY;

      // private static final double kCoupleRatio = 3.5714285714285716;
      private static final double kCoupleRatio = 0.0;

      public static final double kDriveGearRatio = 8.142857142857142;
      public static final double kSteerGearRatio = 21.428571428571427;
      // private static final double kWheelRadiusInches = 1.840; //Direction of resistence
      private static final double kWheelRadiusInches = 1.967; //Direction of less-resistence
      // private static final double kWheelRadiusInches = 1.925; //Comp

      public static final double WHEEL_RADIUS = Units.inchesToMeters(kWheelRadiusInches);

      private static final boolean kSteerMotorReversed = true;
      private static final boolean kInvertLeftSide = false;
      private static final boolean kInvertRightSide = true;

      // These are only used for simulation
      private static final double kSteerInertia = 0.00001;
      private static final double kDriveInertia = 0.001;
      // Simulated voltage necessary to overcome friction
      private static final double kSteerFrictionVoltage = 0.25;
      private static final double kDriveFrictionVoltage = 0.25;

      private static final SwerveModuleConstantsFactory ConstantCreator0 = new SwerveModuleConstantsFactory()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withWheelRadius(kWheelRadiusInches)
              .withSlipCurrent(kSlipCurrentA)
              .withSteerMotorGains(steerGains0)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(kCoupleRatio)
              .withSteerMotorInverted(kSteerMotorReversed);

        private static final SwerveModuleConstantsFactory ConstantCreator1 = new SwerveModuleConstantsFactory()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withWheelRadius(kWheelRadiusInches)
              .withSlipCurrent(kSlipCurrentA)
              .withSteerMotorGains(steerGains1)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(kCoupleRatio)
              .withSteerMotorInverted(kSteerMotorReversed);
        
        private static final SwerveModuleConstantsFactory ConstantCreator2 = new SwerveModuleConstantsFactory()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withWheelRadius(kWheelRadiusInches)
              .withSlipCurrent(kSlipCurrentA)
              .withSteerMotorGains(steerGains2)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(kCoupleRatio)
              .withSteerMotorInverted(kSteerMotorReversed);

        private static final SwerveModuleConstantsFactory ConstantCreator3 = new SwerveModuleConstantsFactory()
              .withDriveMotorGearRatio(kDriveGearRatio)
              .withSteerMotorGearRatio(kSteerGearRatio)
              .withWheelRadius(kWheelRadiusInches)
              .withSlipCurrent(kSlipCurrentA)
              .withSteerMotorGains(steerGains3)
              .withDriveMotorGains(driveGains)
              .withSteerMotorClosedLoopOutput(steerClosedLoopOutput)
              .withDriveMotorClosedLoopOutput(driveClosedLoopOutput)
              .withSpeedAt12VoltsMps(kSpeedAt12VoltsMps)
              .withSteerInertia(kSteerInertia)
              .withDriveInertia(kDriveInertia)
              .withSteerFrictionVoltage(kSteerFrictionVoltage)
              .withDriveFrictionVoltage(kDriveFrictionVoltage)
              .withFeedbackSource(SteerFeedbackType.FusedCANcoder)
              .withCouplingGearRatio(kCoupleRatio)
              .withSteerMotorInverted(kSteerMotorReversed);

      // Front Left
      private static final int kFrontLeftDriveMotorId = 5;
      private static final int kFrontLeftSteerMotorId = 1;
      private static final int kFrontLeftEncoderId = 1;
      private static final double kFrontLeftEncoderOffset = 0.194336;

      private static final double kFrontLeftXPosInches = 10.375;
      private static final double kFrontLeftYPosInches = 10.375;

      // Front Right
      private static final int kFrontRightDriveMotorId = 4;
      private static final int kFrontRightSteerMotorId = 0;
      private static final int kFrontRightEncoderId = 0;
      private static final double kFrontRightEncoderOffset = 0.043701 - 0.5;

      private static final double kFrontRightXPosInches = 10.375;
      private static final double kFrontRightYPosInches = -10.375;

      // Back Left
      private static final int kBackLeftDriveMotorId = 6;
      private static final int kBackLeftSteerMotorId = 2;
      private static final int kBackLeftEncoderId = 2;
      private static final double kBackLeftEncoderOffset = 0.286133 + 0.5;

      private static final double kBackLeftXPosInches = -10.375;
      private static final double kBackLeftYPosInches = 10.375;

      // Back Right
      private static final int kBackRightDriveMotorId = 7;
      private static final int kBackRightSteerMotorId = 3;
      private static final int kBackRightEncoderId = 3;
      private static final double kBackRightEncoderOffset = -0.109863 + 0.5;

      private static final double kBackRightXPosInches = -10.375;
      private static final double kBackRightYPosInches = -10.375;

      public static final SwerveModuleConstants MOD1_CONSTANTS = ConstantCreator1.createModuleConstants(
              kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset, Units.inchesToMeters(kFrontLeftXPosInches), Units.inchesToMeters(kFrontLeftYPosInches), kInvertLeftSide);
      public static final SwerveModuleConstants MOD0_CONSTANTS = ConstantCreator0.createModuleConstants(
              kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset, Units.inchesToMeters(kFrontRightXPosInches), Units.inchesToMeters(kFrontRightYPosInches), kInvertRightSide);
      public static final SwerveModuleConstants MOD2_CONSTANTS = ConstantCreator2.createModuleConstants(
              kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset, Units.inchesToMeters(kBackLeftXPosInches), Units.inchesToMeters(kBackLeftYPosInches), kInvertLeftSide);
      public static final SwerveModuleConstants MOD3_CONSTANTS = ConstantCreator3.createModuleConstants(
              kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset, Units.inchesToMeters(kBackRightXPosInches), Units.inchesToMeters(kBackRightYPosInches), kInvertRightSide);
  }

  public static final class Joystick {
    public static final int LEFT_JOYSTICK_PORT = 1;
    public static final int RIGHT_JOYSTICK_PORT = 0;
    public static final int OPERATOR_PORT = 2;

    public static final double MOVE_DEAD_ZONE = 0.18;
    public static final double TURN_DEAD_ZONE = 0.1;

    public static final double ANGLE_DEAD_ZONE = Constants.TAU / 72;

    public static final double MOVE_SENSITIVITY = 1.5;
    public static final double TURN_SENSITIVITY = 1;
  }

  public static final class Misc {

    public static final int GYRO_PORT = 0;

  }

}
