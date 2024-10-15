// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.base.subsystems.ArmAction;

public class Robot_arm extends SubsystemBase {
  Pose3d base;
  Pose3d arm1;
  Pose3d arm2;
  Pose2d arm_pose;
  public static final double armlengh = 0.9652;
  public static final double baselengh = 0.1651;
  public double speedfactor = 0.02;
  int count = 0;
  double arm1Angle = 0;
  double arm_angle;

  double arm1_endpoint_x;
  double arm1_endpoint_z;

  PIDController m_PidController;

  Supplier<ArmAction> m_ArmAction;

  double setx;
  double sety;
  double setz;

  double buffer = 0.001;
  double pidSpeedB;
  double pidSpeedA1;
  double pidSpeedA2;

  Robot m_robot;

  // 7s 1 rotation

  /** Creates a new Robot_arm. */
  public Robot_arm(Supplier<ArmAction> armAction, Robot robot) {
    arm_pose = new Pose2d();
    base = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));
    arm1 = new Pose3d(0, 0.1524, 0.1016, new Rotation3d(0, 0, 0));
    arm2 = new Pose3d(0, 0, 0, new Rotation3d(0, 0, 0));

    m_robot = robot;

    m_ArmAction = armAction;
    SmartDashboard.putNumber("armAngle", arm_angle);
    // SmartDashboard.putNumber("setPointX", 0);
    // SmartDashboard.putNumber("setPointy", 0);

    m_PidController = new PIDController(31.0, 0.0, 0.85);
    m_PidController.setTolerance(0.02);

    double arm1Angle = 0;
    SmartDashboard.putNumber("setx", 0);
    SmartDashboard.putNumber("sety", 0);
    SmartDashboard.putNumber("setz", 0);
  }

  @Override
  public void periodic() {
    // arm1 = new Pose3d();
    // arm1 = new Pose3d(0, 0.1524, 0.1016, new Rotation3d(0,
    // SmartDashboard.getNumber("armAngle", 0.0), 0));

    setx = SmartDashboard.getNumber("setx", 0);
    sety = SmartDashboard.getNumber("sety", 0);
    setz = SmartDashboard.getNumber("setz", 0) - 0.127;

    arm1 = new Pose3d(0, baselengh, 0.1016, arm1.getRotation());

    // (base.getRotation().getZ() - Math.PI / 2)

    // arm2 = new Pose3d(Math.sin(arm1.getRotation().getY()) * armlengh, 0,
    // Math.cos(arm1.getRotation().getY()) * armlengh + 0.1016,
    // new Rotation3d(0, 0, arm1.getRotation().getZ()));

    arm1_endpoint_x = Math.sin(arm1Angle) * armlengh;
    arm1_endpoint_z = Math.cos(arm1Angle) * armlengh;

    arm2 =
        new Pose3d(
            arm1_endpoint_x,
            0.0127,
            arm1_endpoint_z + 0.1016,
            arm2.getRotation());
    // 32s

    // arm2 = arm2.plus(new Transform3d(0, 0, 0,
    // new Rotation3d(0, speedfactor, 0)));

    // // arm1 = arm1.plus(new Transform3d(0, 0, 0,
    // // new Rotation3d(0, speedfactor, 0)));

    // base = base.plus(new Transform3d(0, 0, 0,
    // new Rotation3d(0, 0, speedfactor)));

    Logger.recordOutput("arm_pose", arm_pose);
    Logger.recordOutput("base", base);
    Logger.recordOutput("arm1", arm1);
    Logger.recordOutput("arm2", arm2);
    Logger.recordOutput("armangle", arm1.getRotation().getY());
    Logger.recordOutput("armz", Math.sin(arm1.getRotation().getY()) * armlengh);

    Logger.recordOutput("armX", arm1.getTranslation().getX());
    Logger.recordOutput("armY", arm1.getTranslation().getY());
    Logger.recordOutput("armZ", arm1.getTranslation().getZ());



    Logger.recordOutput("pidspeedB", pidSpeedB);
    Logger.recordOutput("pidspeedA1", pidSpeedA1);
    Logger.recordOutput("pidspeedA2", pidSpeedA2);

    // System.out.println(m_ArmAction.get());
    switch (m_ArmAction.get()) {
      case DEFAULT:
        base = base.plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));

        arm1 = arm1.plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));
        break;
      case BASER:
        base = base.plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, speedfactor)));
        break;
      case BASEL:
        base = base.plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, -speedfactor)));
        break;
      case ARMC:
        arm1 = arm1.plus(new Transform3d(0, 0, 0, new Rotation3d(0, speedfactor, 0)));

        arm1Angle += speedfactor;
        break;
      case ARMCC:
        arm1 = arm1.plus(new Transform3d(0, 0, 0, new Rotation3d(0, -speedfactor, 0)));

        arm1Angle -= speedfactor;
        break;
      case ARM2CC:
        arm2 = arm2.plus(new Transform3d(0, 0, 0, new Rotation3d(0, speedfactor, 0)));

        break;
      case ARM2C:
        arm2 = arm2.plus(new Transform3d(0, 0, 0, new Rotation3d(0, -speedfactor, 0)));

        break;
      case POS:
        // x0, y0, z 0.127
        // 1.9304

         if (isInRange()) {
          m_robot.setArmAction(ArmAction.DEFAULT);
          System.out.println("noooooo");
        }

        // base turn
        //make the whole arm line up to the point
        pidSpeedB = m_PidController.calculate(base.getZ(), Math.atan(setx / sety)) * 0.005;

        //where
        // l = distance between the two center point
        double l = Math.sqrt(Math.pow(setx, 2) + Math.pow((setz - 0.1016), 2));
        // D = intersection between connect the the two point where two circle cut and
            // the line that connect the two center of the circle
          //d is distance from D to one of the circle
        double d = (Math.pow(l, 2) 
                  - Math.pow(armlengh, 2) 
                  + Math.pow(armlengh, 2)) / 2 / l;

        //y is distance from D to one of the intersection
        double y = Math.sqrt(Math.pow(armlengh, 2) - Math.pow(d, 2) );

        double arm1_targetAngle = Math.atan(y/d);

        double arm2_targetAngle = Math.atan((setx - arm1_endpoint_x)/ (setz - arm1_endpoint_z));

        pidSpeedA1 = m_PidController.calculate(
                              base.getRotation().getY(), arm1_targetAngle) * 0.005;

        pidSpeedA2 = m_PidController.calculate(
                              base.getRotation().getY(), arm2_targetAngle) * 0.005;


        if (atSetpoint()){
          stopAll();
          break;
        }
                            
        // base = base.plus(
        //               new Transform3d(0, 0, 0, 
        //               new Rotation3d(0, 0, pidSpeedB)));
        // arm1 = arm1.plus(
        //               new Transform3d(0, 0, 0, 
        //               new Rotation3d(0, pidSpeedA1, 0)));

        // arm2 = arm2.plus(
        //               new Transform3d(0, 0, 0, 
        //               new Rotation3d(0, pidSpeedA2, 0)));
        
        
        // base = base.plus(new Transform3d(0, 0, 0,
        //     new Rotation3d(0, 0, -pidSpeedB)));

        //  pidSpeedB = m_PidController.calculate(
        //     arm1Angle % (2*Math.PI), Math.atan(setx / sety)) * 0.005;

        // 0.9652^2 = (z-setz)^2 + (x-setx)^2
        // 0.9652^2 = (z-0.127)^2 + (x)^2

        // z[-2(setz)+2(0.127)] + x[-2(setx)] + setx^2 + setz^ -0.127^2

        // dxy Math.pow(Math.sqrt(setx*setx + sety * sety), 2)
        // dz Math.sqrt(Math.pow(Math.sqrt(setx*setx + sety * sety), 2) + setz * setz)

      break;
    }

    // This method will be called once per scheduler run
  }

  public void baseR() {
    speedfactor = 0;
    // base = base.plus(new Transform3d(0, 0, 0,
    // new Rotation3d(0, 0, speedfactor)));
  }

  public void baseL() {
    base = base.plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, -speedfactor)));
  }

  public void arm1Up() {
    arm1 = arm1.plus(new Transform3d(0, 0, 0, new Rotation3d(0, -speedfactor, 0)));
  }

  public void arm1Down() {
    arm1 = arm1.plus(new Transform3d(0, 0, 0, new Rotation3d(0, -speedfactor, 0)));
  }

  public void stopAll() {
    base = base.plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));

    arm1 = arm1.plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));

    arm2 = arm2.plus(new Transform3d(0, 0, 0, new Rotation3d(0, 0, 0)));

  }

  public boolean isInRange() {

    return Math.sqrt(Math.pow(Math.sqrt(setx * setx + sety * sety), 2) + (setz - 0.1016)) * (setz - 0.1016) <= 0.9652;
  }

  public boolean atSetpoint() {
    double totArmxyL =
        Math.sin(arm1Angle) * armlengh + Math.sin(arm2.getRotation().getY()) * armlengh;
    if (Math.sin(totArmxyL) <= setx + buffer
        && Math.sin(totArmxyL) >= setx - buffer
        && Math.cos(totArmxyL) <= sety + buffer
        && Math.cos(totArmxyL) >= sety - buffer
        && Math.cos(arm1Angle) * armlengh + Math.cos(arm2.getRotation().getY()) * armlengh + 0.1016
            <= setz + buffer
        && Math.cos(arm1Angle) * armlengh + Math.cos(arm2.getRotation().getY()) * armlengh + 0.1016
            >= setz - buffer) {

              return true;
            }

       return false;
  }
}
