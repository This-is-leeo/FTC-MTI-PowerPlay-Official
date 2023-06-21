
package org.firstinspires.ftc.teamcode.auton;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.C;
import org.firstinspires.ftc.teamcode.analogDistanceDriver;
import org.firstinspires.ftc.teamcode.drivetrain.Drivetrain;
import org.firstinspires.ftc.teamcode.input.AsyncThreaded;

import org.firstinspires.ftc.teamcode.output.motorimpl.DoesntResetDcMotorExMotor;
import org.firstinspires.ftc.teamcode.output.motorimpl.ServoMotor;
import org.firstinspires.ftc.teamcode.pid.Pid;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.utils.M;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicBoolean;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class leftSideSplineAuto extends LinearOpMode {

    analogDistanceDriver Sensor2;
    double distance;

    Pose2d initialPosition = new Pose2d(-36, -60, Math.toRadians(-90));
    Pose2d secondPosition = new Pose2d(-36, -24, Math.toRadians(-90));
    Pose2d finalPosition = new Pose2d(-24, -10, Math.toRadians(180));

    OpenCvCamera camera;
    AprilTagDetectionPipeline aprilTagDetectionPipeline;
    AprilTagDetection tagOfInterest = null;
    int randomization = 1;
    double fx = 578.272;
    double fy = 578.272;
    double tagsize = 0.166;
    double cx = 402.145;
    double cy = 221.506;
    int LEFT = 1;
    int MIDDLE = 2;
    int RIGHT = 3;
    TrajectorySequence traj;
    Trajectory traj1;
    Trajectory park1;
    Trajectory park2;
    Trajectory park3;
    private final int state = 0;
    private AtomicBoolean test;
    private AsyncThreaded secondThread;
    AsyncThreaded standardLoopThread;
    int conesStacked = 6;
    boolean hasIntaked = false;
    boolean finishedStacking = false;

    //
    private TouchSensor turretSensor;
    private Pid linSlidePid;
    private Pid pitchPid;
    private Pid turretPid;
    private Drivetrain drivetrain;
    private DoesntResetDcMotorExMotor turret;
    private DoesntResetDcMotorExMotor pitch;
    private DoesntResetDcMotorExMotor leftLinSlide;
    private DoesntResetDcMotorExMotor rightLinSlide;
    private ServoMotor claw;
    private ServoMotor frontArm;
    private ServoMotor deposit;
    private ServoMotor latch;
    private ServoMotor leftArm;
    private ServoMotor rightArm;
    int a;
    private static final double turretP = 3.2;
    private static final double turretI = 1.2;
    private static final double turretD = 0;

    private static final double pitchP = 3.2;
    private static final double pitchI = 1.2;
    private static final double pitchD = 0;

    private static final double slidesP = 3.2;
    private static final double slidesI = 1.2;
    private static final double slidesD = 0;

    //Variable
    private int linSlidePosition = 0;
    private int depositPosition = 0;
    private final int clawPosition = 0;
    private final int latchPosition = 0;
    private int frontArmPosition = 1;
    private int armPosition = 2;
    private int pitchPosition;
    private final double[] linSlidePositions = {0,0.5,0.9};

    private final double[] clawPositions = { 0.0, 1.0};
    private final double[] frontArmPositions = {0,0.93,0.6};
    //Calculation variable!
    private double targetPitchPosition;
    private double targetTurretPosition;
    private double targetLinSlidePosition = this.linSlidePositions[linSlidePosition];
    private double targetFrontArmPosition = C.frontArmPositions[frontArmPosition];
    private double targetDepositPosition = C.depositPositions[depositPosition];
    private double targetArmPosition = 0.8;

    private boolean clawOpen = true;
    private boolean latchEngaged = true;
    private final double firstScorePosition = 0.31;
    private final double secondScorePosition = 0.72;
    SampleMecanumDrive drive;
    Pose2d poseEstimate;
    private boolean pitchTS;
    private double linSlidePower;

    private BNO055IMU imu;

    private AsyncThreaded scoring;
    private void initDrivetrain() {
        drive = new SampleMecanumDrive(hardwareMap);
        drive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        drive.getLocalizer().setPoseEstimate(new Pose2d(-36,-60,Math.toRadians(-90)));
    }
    private void initMotor() {
        this.pitch = new DoesntResetDcMotorExMotor(hardwareMap.get(DcMotorEx.class, "pitch"))
                .setLowerBound(C.pitchLB)
                .setUpperBound(C.pitchUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.turret = new DoesntResetDcMotorExMotor(hardwareMap.get(DcMotorEx.class, "turret"))
                .setLowerBound(C.turretLB)
                .setUpperBound(C.turretUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.leftLinSlide = new DoesntResetDcMotorExMotor(hardwareMap.get(DcMotorEx.class, "leftSlide"))
                .setLowerBound(C.linSlideLB)
                .setUpperBound(C.linSlideUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        this.rightLinSlide = new DoesntResetDcMotorExMotor(hardwareMap.get(DcMotorEx.class, "rightSlide"))
                .setLowerBound(C.linSlideLB)
                .setUpperBound(C.linSlideUB)
                .setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    private void initPID() {
        //tune later cuh
        this.linSlidePid = new Pid(new Pid.Coefficients(slidesP, slidesI, slidesD),
                () -> this.targetLinSlidePosition - this.leftLinSlide.getCurrentPosition(),
                factor -> {
                    this.leftLinSlide.setPower(M.clamp(-factor, -1, 0.6));
                    this.rightLinSlide.setPower(M.clamp(-factor, -1, 0.6));
                });
        this.pitchPid = new Pid(new Pid.Coefficients(pitchP, pitchI, pitchD),
                () -> this.targetPitchPosition - this.pitch.getCurrentPosition(),
                factor -> {
                    this.pitch.setPower(M.clamp(-factor, - 0.5, 0.5));
                });
        this.turretPid = new Pid(new Pid.Coefficients(turretP, turretI, turretD),
                () -> this.targetTurretPosition - this.turret.getCurrentPosition(),
                factor -> {
                    this.turret.setPower(M.clamp(factor, -.8, 0.8));
                });
    }
    private void initServo() {
        this.deposit = new ServoMotor(hardwareMap.get(Servo.class, "deposit"))
                .setLowerBound(C.depositLB)
                .setUpperBound(C.depositUB);
        this.latch = new ServoMotor(hardwareMap.get(Servo.class, "latch"))
                .setLowerBound(C.latchLB)
                .setUpperBound(C.latchUB);
        this.claw = new ServoMotor(hardwareMap.get(Servo.class, "claw"))
                .setLowerBound(C.clawLB)
                .setUpperBound(C.clawUB);
        this.leftArm = new ServoMotor(hardwareMap.get(Servo.class, "leftLinkage"))
                .setLowerBound(C.leftArmLB)
                .setUpperBound(C.leftArmUB);
        this.rightArm = new ServoMotor(hardwareMap.get(Servo.class, "rightLinkage"))
                .setLowerBound(C.rightArmLB)
                .setUpperBound(C.rightArmUB);
        this.frontArm = new ServoMotor(hardwareMap.get(Servo.class, "frontArm"))
                .setLowerBound(C.frontArmLB)
                .setUpperBound(C.frontArmUB);
    }
    private void initSensor() {
        Sensor2 = new analogDistanceDriver(hardwareMap.get(AnalogInput.class, "name"));
    }
    private void initAll() {
        this.initPID();
        this.initMotor();
        this.initServo();
        this.initDrivetrain();
        this.initPosition();
        this.initSensor();
        this.initAsync();
    }
    private void updateServo() {
        this.deposit.update();
        this.claw.update();
        this.latch.update();
        this.rightArm.update();
        this.leftArm.update();
        this.frontArm.update();
    }
    private void updateMotor() {
        this.linSlidePid.update();
        this.pitchPid.update();
        this.turretPid.update();
        this.leftLinSlide.update();
        this.rightLinSlide.update();
        this.pitch.update();
        this.turret.update();
    }
    private void updateDrivetrain() {
//        poseEstimate = drive.getPoseEstimate();
        drive.update();
    }

    private void updateTelemetry() {
        telemetry.addData("pitch TS", this.pitchTS);
        telemetry.addData("latchEngaged" , this.latchEngaged);
        telemetry.addData("turret pos", this.turret.getCurrentPosition());
        telemetry.update();
    }
    private void updateVariable() {
        targetLinSlidePosition = M.clamp(this.linSlidePositions[this.linSlidePosition] + this.linSlidePositions[this.linSlidePosition]*(0.4 -this.targetPitchPosition) ,0,1);
    }
    private void initPosition() {
        moveArm(0);
        this.targetFrontArmPosition = 0.9;
    }
    private void intakeBack(){
        this.clawOpen = false;
        sleep(500);
        frontArmPosition = 1;
        moveFrontArm();
        hasIntaked = true;
        sleep(100);
        targetArmPosition = 0;
    }
    private void greatReset(){
        this.linSlidePosition = 0;
        //pause
        long start = System.currentTimeMillis();
        this.depositPosition = 0;
        moveDeposit();
        latchEngaged = false;
        movePitch(0.27);
        this.targetTurretPosition = 0.5;
    }

    private void dump(){
        depositPosition = 2;
    }
    private void moveArm(int pos){
        if(pos >= 0 && pos < 10) this.targetArmPosition = (C.armPositions[armPosition]);
        else this.targetArmPosition = 0;
    }
    private void intakeOut(){
        this.clawOpen = true;
        frontArmPosition = 0;
        moveFrontArm();
        moveArm(armPosition);
    }
    private void moveDeposit(){
        this.targetDepositPosition = (C.depositPositions[depositPosition]);
    }
    private void moveFrontArm(){
        targetFrontArmPosition = this.frontArmPositions[frontArmPosition] - (0.1*this.frontArmPositions[frontArmPosition]*M.clamp(this.pitch.getCurrentPosition(),0.5,1.5));
    }
    private void movePitch(double position){
        this.targetPitchPosition = M.clamp(position , 0.1, 1);
    }
    private void updatePosition() {
        this.frontArm.setPosition(targetFrontArmPosition);
        this.leftArm.setPosition(targetArmPosition);
        this.rightArm.setPosition(targetArmPosition);
        if(latchEngaged)this.latch.setPosition(1);
        else this.latch.setPosition(0);
        if(clawOpen)this.claw.setPosition(0);
        else this.claw.setPosition(0.6);
        this.deposit.setPosition(targetDepositPosition);
    }

    private void updateAll() { //order: DT, Var, sensor, Control, position, motor. servo, telemetry.
        this.updateVariable();
        this.updatePosition();
        this.updateMotor();
        this.updateServo();
        this.updateTelemetry();
        //this.updateDrivetrain();
    }
    private void preIntakeMode(){
        this.targetFrontArmPosition = 0.6;
    }
    private void linSlideUp(){
        this.linSlidePosition = 2;
    }
    private void linSlideReset(){
        if(latchEngaged)this.latchEngaged = false;
        if(targetFrontArmPosition > 0.8) {
            targetFrontArmPosition = 0.7;
        }
        this.linSlidePosition = 0;
        this.depositPosition = 0;
        moveDeposit();
    }
    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        return radians;//its radians you dumbass
    }

    public void runMainLoop(){
            this.score();
            this.preIntakeMode(5);
            while (!linSlideCheck()) {
                sleep(1);
            }
            latchEngaged = false;
            sleep(450);
            this.armPosition = 9;
            linSlideReset();
            intakeOut(C.getTargetFrontArmPosition(5));
            sleep(350);
            this.targetTurretPosition = 0.5;
            sleep(200);
            for(int j = 4; j>=0; j--) {
                sleep(275);
                this.targetTurretPosition = 0.5;
                intakeBack();
                targetPitchPosition = 0.55;
                sleep(1000);
                this.clawOpen = true;
                sleep(450);

                targetPitchPosition = 0.36;
                targetTurretPosition = 0.83;
                sleep(150);
                if(j == 0) break;
                preIntakeMode(j);
                this.score();
                while(!linSlideCheck()){
                    sleep(1);
                }
                latchEngaged = false;
                sleep(350);
                linSlideReset();
                intakeOut(C.getTargetFrontArmPosition(j));
                sleep(400);
            }
            preIntakeMode();
            this.score();
            while(!linSlideCheck()){
                sleep(1);
            }
            latchEngaged = false;
            sleep(350);
            linSlideReset();
            sleep(450);
            greatReset();
            targetPitchPosition = 0.7;
            targetTurretPosition = 0.23;
            finishedStacking = true;
    } 

    void runOtherLoop(int pos){
        String s = "Test";
//        String ss = toString(pos);
        telemetry.addLine(s);
    }


    @Override
    public void runOpMode() throws InterruptedException {

        while(opModeInInit()) {
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
            camera.setPipeline(aprilTagDetectionPipeline);
            camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
            {
                @Override
                public void onOpened()
                {
                    camera.startStreaming(800,448, OpenCvCameraRotation.UPRIGHT);
                }

                @Override
                public void onError(int errorCode)
                {

                }
            });
            ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
            if (currentDetections.size() != 0) {
                boolean tagFound = false;
                for (AprilTagDetection tag : currentDetections) {
                    if (tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                        tagOfInterest = tag;
                        tagFound = true;
                        break;
                    }
                }
                if (tagOfInterest == null || tagOfInterest.id == LEFT) {
                    telemetry.addLine("LEFT");
                    randomization = 1;
                } else if (tagOfInterest.id == MIDDLE) {
                    telemetry.addLine("MIDDLE");
                    randomization = 2;
                } else {
                    telemetry.addLine("RIGHT");
                    randomization = 3;
                }
                telemetry.update();
            }
            telemetry.update();
            this.initAll();
            this.initDrivetrain();

            drive.setPoseEstimate(new Pose2d(-36, -60, Math.toRadians(-90)));
            traj = drive.trajectorySequenceBuilder(drive.getPoseEstimate())
                    .setReversed(true)
                    .splineToSplineHeading(secondPosition, calculateTangent(initialPosition,secondPosition))
                    .splineToSplineHeading(finalPosition, Math.toRadians(0))
//                    .lineToLinearHeading(new Pose2d(-36, -2, Math.toRadians(-90)))
                    .build();
            traj1 = drive.trajectoryBuilder(traj.end())
                    .lineToLinearHeading(new Pose2d(-38, -10, Math.toRadians(-180)))
                    .build();
            park1 = drive.trajectoryBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(-11.5, -12, Math.toRadians(-180)))
                    .build();
            park2 = drive.trajectoryBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(-36, -12.1, Math.toRadians(-90)))
                    .build();
            park3 = drive.trajectoryBuilder(traj1.end())
                    .lineToLinearHeading(new Pose2d(-59, -12, Math.toRadians(-180)))
                    .build();
            if (isStopRequested()) return;
            waitForStart();
        }
        if(opModeIsActive()) {
            AsyncThreaded.stopped = false;
            this.initAsync();

            this.targetPitchPosition = 0.7;
            this.targetTurretPosition = 0.5;
            this.secondThread.run();
            drive.followTrajectorySequence(traj);
            this.targetPitchPosition = 0.36;
            this.targetTurretPosition = 0.8;
//            drive.followTrajectory(traj1);

            //TODO here:
            standardLoopThread = new AsyncThreaded(() -> {
                }).then(() -> {
                    runMainLoop();
                });
            AsyncThreaded obsticleAvoidanceThrad = new AsyncThreaded(() -> {}).then(() -> {
                    while (!finishedStacking && this.opModeInInit() || this.opModeIsActive() && !AsyncThreaded.stopped) {
                        
                        //TODO ping for detection
                        distance = Sensor2.getDistance();
                        telemetry.addData("distance", distance);
                        telemetry.update();

                        if(distance < 10){
                            standardLoopThread.stop();
                            //TODO reset everything


                            for(int i = conesStacked; i > 0; i--)
                                runOtherLoop(i);
                            finishedStacking = true;
                            break;
                        }
                    }
                });

            while(true) if(finishedStacking == true) break;

            if(randomization == 3) {
                drive.followTrajectory(park3);
            } else if(randomization ==2){
                drive.followTrajectory(park2);
            } else if(randomization ==1) {
                drive.followTrajectory(park1);
            }
        }

    }
    private void shake(){
        this.targetDepositPosition = this.targetDepositPosition + 0.2;
        sleep(100);
        this.targetDepositPosition = this.targetDepositPosition - 0.2;
    }

    private boolean linSlideCheck(){
        return Math.abs(this.leftLinSlide.getCurrentPosition() - this.targetLinSlidePosition) < 0.05;
    }
    private void moveFrontArm1(double position){
        targetFrontArmPosition = position;
    }
    private void intakeOut(double position){
        if(position < 0) position = 0;
        this.clawOpen = true;
//        frontArmPosition = 0;
        moveFrontArm1(position);
        moveArm(armPosition);
    }
    private void score(){
        linSlideUp();
        sleep(100);
        sleep(450);
        //this.latchEngaged = true;
        depositPosition = 2;
        this.targetDepositPosition = (C.depositPositions[depositPosition]) - 0.17;
        conesStacked--;
        hasIntaked = true;
        sleep(250);
    }

    private void preIntakeMode(int i){
        this.targetArmPosition = 0.3;
        this.targetFrontArmPosition = C.getTargetFrontArmPosition(i);
    }
    public static double calculateTangent(Pose2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Pose2d initialPosition, Vector2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }
    public static double calculateTangent(Vector2d initialPosition, Pose2d finalPosition) {
        double xd = initialPosition.getX() - finalPosition.getX();
        double yd = initialPosition.getY() - finalPosition.getY();
        return Math.atan2(yd,xd) - Math.PI;
    }

    private void initAsync() {
        secondThread = new AsyncThreaded(() -> {
        })
                .then(() -> {
                    while (this.opModeInInit() || this.opModeIsActive() && !AsyncThreaded.stopped) {
                        Pose2d poseEstimate = drive.getPoseEstimate();
                        this.updateAll(); //TODO Add variable Change in updateVariable();


                    }
                });
    }

}