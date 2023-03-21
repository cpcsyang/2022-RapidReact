// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivers;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.PigeonIMU.CalibrationMode;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import edu.wpi.first.wpilibj.interfaces.Gyro;

public class Pigeon implements Gyro, Accelerometer {

    PigeonIMU pigeon;
    
    // Gyro variables
    double angle;
    double timeStamp;
    double[] lastAngleTime = new double[2];
    double[] currentAngleTime = new double[2];
    double rotateRate;

    // Accelerometer variables
    Range accelRange;
    double[] tiltAngle = new double[3];
    double accelX;
    double accelY;
    double accelZ;
        
    // create new pigeon
    public Pigeon(int id) {
        pigeon = new PigeonIMU(id);
    }

    // enter boot-calibration (gyro and temp), can also be done in phoenix tuner
    @Override
    public void calibrate() {
        pigeon.enterCalibrationMode(CalibrationMode.BootTareGyroAccel);
        
    }

    // get gyro
    @Override
    public double getAngle() {
        angle = pigeon.getFusedHeading();
        return angle;
    }

    // reset gyro
    @Override
    public void reset() {
        pigeon.setFusedHeading(180);
    }

    public void invert() {
        pigeon.setFusedHeading(0);
    }

    // gyro rate in degree/s
    // call in drivebase periodic to get continuous update
    // assume constant rotation speed
    @Override
    public double getRate() {
        timeStamp = Timer.getFPGATimestamp();
        angle = getAngle();
        currentAngleTime[0] = angle;
        currentAngleTime[1] = timeStamp;

        rotateRate = (currentAngleTime[0] - lastAngleTime[0]) / (currentAngleTime[1] - lastAngleTime[1]);

        return rotateRate;
    }

    // set accelerometer range (2g, 16 bytes for pigeon)
    public void setRange(Range accelRange) {
        this.accelRange = accelRange;
    }

    // x acceleration in g
    @Override
    public double getX() {
        pigeon.getAccelerometerAngles(tiltAngle);
        accelX = Math.sin(Math.toRadians(tiltAngle[0]));
        return accelX;
    }

    // y acceleration in g
    @Override
    public double getY() {
        pigeon.getAccelerometerAngles(tiltAngle);
        accelY = Math.sin(Math.toRadians(tiltAngle[1]));
        return accelY;
    }

    // z acceleration in g
    @Override
    public double getZ() {
        pigeon.getAccelerometerAngles(tiltAngle);
        accelZ = Math.sin(Math.toRadians(tiltAngle[2]));
        return accelZ;
    }

    // specify exception
    @Override
    public void close() throws Exception {} 
}