// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;
import frc.robot.generated.TunerConstants;


import com.ctre.phoenix6.configs.GyroTrimConfigs;
import com.ctre.phoenix6.hardware.Pigeon2;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class Ll extends SubsystemBase {
  private final Field2d field2d = new Field2d();
  private final CommandSwerveDrivetrain m_drive = TunerConstants.createDrivetrain();
  private Pose2d pose2d = new Pose2d();
  private PoseEstimate robPoseEstimate = new PoseEstimate();
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      new Translation2d(0.32385, 0.32385),
      new Translation2d(0.32385, -0.32385),
      new Translation2d(-0.32385, 0.32385),
      new Translation2d(-0.32385, -0.32385));
  private final SwerveDrivePoseEstimator poseEstimator = new SwerveDrivePoseEstimator(kinematics, getGyroRotation(), null,
      pose2d);
  

  private final Rotation2d gyroangle = new Rotation2d();
  private final Pigeon2 m_Pigeon2 = new Pigeon2(1);
  private final SwerveModulePosition[] getmodulePositions = new SwerveModulePosition[] {
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition(),
      new SwerveModulePosition() };

  /** Creates a new Version. */
  public Ll(CommandSwerveDrivetrain drive) {
  }

  public void Roboinit() {
    m_Pigeon2.setYaw(0);
  }

  @Override
  public void periodic() {
    m_drive.getRotation3d();
    m_drive.getModuleLocations();
    SmartDashboard.putData("Field", field2d);

    
    gyroangle.getDegrees();
    var mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
    if (mt2.tagCount > 0) {
      poseEstimator.addVisionMeasurement(pose2d, mt2.timestampSeconds);
    }

    // This method will be called once per scheduler run
  }

  public Rotation2d getGyroRotation() {
    return m_Pigeon2.getRotation2d();
  }

}
