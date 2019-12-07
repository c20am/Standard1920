package org.firstinspires.ftc.teamcode.tests;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

public class Gyro {
    BNO055IMU imu;
    Orientation angles;
    double forwardHeading = 0.0;

    public Gyro(BNO055IMU imu) {
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);
        this.imu = imu;
    }

    public double heading() {
        update();
        if(angles != null) {
            double h = angles.firstAngle - forwardHeading;
            if (h < -Math.PI) h += 2*Math.PI;
            if (h > Math.PI) h -= 2*Math.PI;
            return h;
        } else {
            return -3*Math.PI;
        }
    }

    public void update() {
        angles = imu.getAngularOrientation().toAxesReference(AxesReference.INTRINSIC).toAxesOrder(AxesOrder.ZYX);
    }

    public void resetHeading() {
        if(angles != null)
            forwardHeading = angles.firstAngle;
    }

}