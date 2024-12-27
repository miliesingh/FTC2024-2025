package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
@Autonomous(name = "distanceTestProgram", group = "Auto2024")
public class distanceTestProgram extends LinearOpMode{
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;
    private DistanceSensor sensorRange;
    private Servo clawWristServo = null;


    public void driveForward(double power){
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        leftFront.setPower(-power);
        rightBack.setPower(-power);
    }
    public void stopRobot(){
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
    }
    @Override
    public void runOpMode(){
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        clawWristServo = hardwareMap.get(Servo.class, "clawWristServo");
        sensorRange = hardwareMap.get(DistanceSensor.class, "distanceSensor");

        waitForStart();
        runtime.reset();

        double distance = sensorRange.getDistance(DistanceUnit.CM);
        clawWristServo.setPosition(1.0);
        if(opModeIsActive()){
            telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
            while (distance > 20){
                distance = sensorRange.getDistance(DistanceUnit.CM);
                driveForward(0.2);
                telemetry.update();
            }
            stopRobot();
        }
    }
}

