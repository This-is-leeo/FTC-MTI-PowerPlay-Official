package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import org.firstinspires.ftc.teamcode.analogDistanceDriver;

@TeleOp
public class sampleDistance extends LinearOpMode{

    analogDistanceDriver Sensor2;
    double distance;

    public void runOpMode() throws InterruptedException {
        Sensor2 = new analogDistanceDriver(hardwareMap.get(AnalogInput.class, "name"));
        waitForStart();
        while(opModeIsActive()) {

            distance = Sensor2.getDistance();
            telemetry.addData("distance", distance);
            telemetry.update();
        }
    }
}
