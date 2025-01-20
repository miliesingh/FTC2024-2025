package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "linearAndDriving", group = "Auto2024")
public class linSlideAndDrivingTest extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    private DcMotor linSlideR = null;
    private DcMotor linSlideL = null;

    public void linSlidesUp(double power){
        linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideL.setPower(power);
        linSlideR.setPower(power);
    }
    public void linSlidesDown(double power){
        linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideL.setPower(power);
        linSlideR.setPower(power);

    }

    public void linSlidesStay(){
        linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideL.setPower(0.05);
        linSlideR.setPower(0.05);
    }

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

    public void LINEAR_SLIDE_DRIVE(float distance_in_in, double power) {
        float ticksPerInch = 450.149432158f;
        float f_ticks = ticksPerInch * distance_in_in;
        int ticks = Math.round(f_ticks);
        //753.1 ticks per revolution
        //1.673 in per revolution (circumference)
        //450.149432158 ticks per in
        if (power > 0) {
            //go up
            linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
            linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            //go down
            linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
            linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        linSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideL.setPower(power);
        linSlideL.setTargetPosition(ticks);
        linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideR.setPower(power);
        linSlideR.setTargetPosition(ticks);
        linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("slide", "> is running to position");
        telemetry.update();
        while ((linSlideR.getCurrentPosition() <= linSlideR.getTargetPosition() - 50) && linSlideL.getCurrentPosition() <= (linSlideL.getTargetPosition() - 50)) {
            //Wait until job is finished
            telemetry.addData("slide", "> is strafing to position");
            telemetry.addData("ticks", ">" +
                    linSlideR.getCurrentPosition() + " need to get to " +
                    linSlideR.getTargetPosition());
            telemetry.addData("slide", "> is strafing to position");
            telemetry.addData("ticks", ">" + linSlideL.getCurrentPosition() + " need to get to " + linSlideL.getTargetPosition());
            telemetry.update();
        }
    }

    public void runOpMode() {
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
        linSlideL = hardwareMap.get(DcMotor.class, "linSlideL");
        linSlideR = hardwareMap.get(DcMotor.class, "linSlideR");
        linSlideL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        linSlideR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        waitForStart();
        runtime.reset();
        linSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(opModeIsActive()){
            float ticksPerInch = 450.149432158f;
            float f_ticks = ticksPerInch * 5;
            int ticks = Math.round(f_ticks);
            linSlideL.setTargetPosition(ticks);
            linSlideR.setTargetPosition(ticks);
            while(true){
                driveForward(0.2);
                if((linSlideR.getCurrentPosition() >= linSlideR.getTargetPosition() - 50) && linSlideL.getCurrentPosition() >= (linSlideL.getTargetPosition() - 50)){
                    break;
                }
                else {
                    linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlidesUp(1.0);
                    linSlideL.setTargetPosition(ticks);
                    linSlideR.setTargetPosition(ticks);
                }

            }
            linSlidesStay();
            sleep(1000);
            ticksPerInch = 450.149432158f;
            f_ticks = ticksPerInch * 2;
            ticks = Math.round(f_ticks);
            linSlideL.setTargetPosition(ticks);
            linSlideR.setTargetPosition(ticks);
            while(true){
                driveForward(0.2);
                if((linSlideR.getCurrentPosition() <= linSlideR.getTargetPosition()) && linSlideL.getCurrentPosition() <= (linSlideL.getTargetPosition())){
                    break;
                }
                else {
                    linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlidesUp(-1.0);
                    linSlideL.setTargetPosition(ticks);
                    linSlideR.setTargetPosition(ticks);
                }

            }
            linSlidesStay();
            sleep(1000);
            ticksPerInch = 450.149432158f;
            f_ticks = ticksPerInch * 7;
            ticks = Math.round(f_ticks);
            linSlideL.setTargetPosition(ticks);
            linSlideR.setTargetPosition(ticks);
            while(true){
                driveForward(0.2);
                if((linSlideR.getCurrentPosition() >= linSlideR.getTargetPosition()) && linSlideL.getCurrentPosition() >= (linSlideL.getTargetPosition())){
                    break;
                }
                else {
                    linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    linSlidesUp(1.0);
                    linSlideL.setTargetPosition(ticks);
                    linSlideR.setTargetPosition(ticks);
                }

            }
            linSlidesStay();
            sleep(9999999);

        }

    }
}
