package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Locale;

// This is far side Blue
@Autonomous(name="basketSide", group="Auto2024")
public class sampleSideAutonomous extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    private DcMotor linSlideR = null;
    private DcMotor linSlideL = null;
    private Servo clawWristServo = null;
    private Servo clawServo = null;


    GoBildaPinpointDriver odo;

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


    //  public void getData() {
    //     sensorRange1.getDistance(DistanceUnit.CM);
    //  sensorRange2.getDistance(DistanceUnit.CM);
    //  }


    public double angleWrap(double radians) {
        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }
        double angle = radians * 180 / Math.PI;
        // keep in mind that the result is in radians
        return angle;
    }

    public void turnLeft(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void strafeRight(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void softTurnLeft(double power) {
        leftBack.setPower(power / 2);
        rightFront.setPower(power);
        leftFront.setPower(power / 2);
        rightBack.setPower(power);
    }

    public void softTurnRight(double power) {
        rightBack.setPower(power / 2);
        leftFront.setPower(power);
        rightFront.setPower(power / 2);
        leftBack.setPower(power);
    }


    public void turnRight(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void stopRobot() {
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
    }

    public void driveForward(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
    }

    public void driveBackwardCorrection(double angle, double power, int distance, double pos) {
        String turn = "";
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        driveForward(power);
//            if (angle <= -5) {
//                softTurnLeft(power);
//                turn = "left";
//            } else if (angle >= 5) {
//                softTurnRight(power);
//                turn = "right";
//            } else if (angle < 5 && angle > -5) {
//                driveForward(power);
//                turn = "forward";
//            } else {
//                stopRobot();
//            }

        telemetry.addData("turn", turn);
        telemetry.update();
    }

    public void driveForwardCorrection(double angle, double power, int distance, double pos) {
        String turn = "";
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        driveForward(-power);
//            if(angle <=-5){
//                softTurnLeft(-power);
//                turn = "left";
//            }
//            else if (angle>= 5) {
//                softTurnRight(-power);
//                turn = "right";
//            }
//            else if (angle <5 && angle > -5) {
//                driveForward(-power);
//                turn = "forward";
//            }
//            else{
//                stopRobot();
//            }

        telemetry.addData("turn", turn);
        telemetry.update();


//        if(yPos > 3){
//            leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//            rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//            rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//            leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//        }
//        if(yPos < 3){
//            leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
//            rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//            rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//            leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//        }
//         if(angle <-5){
//             leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
//             rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//             leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
//             rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//         } else if (angle > 5) {
//             leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//             rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
//             leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//             rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
//         }
//         else {
//             leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
//             rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
//             leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
//             rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
//         }

    }

    public void turnDegrees(double nAngle, double power) {
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            double angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            turnRight(power);
            if (Math.abs(angle) > nAngle) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
    }

    public void driveDistanceForward(double power, int distance, double xPlace) {
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        if (xPlace > distance) {
            if (xPlace > distance / 2) {
                leftBack.setPower(power);
                rightFront.setPower(power);
                leftFront.setPower(power);
                rightBack.setPower(power);
                telemetry.update();
            }
            if (xPlace < distance / 2) {
                leftBack.setPower(power / 2);
                rightFront.setPower(power / 2);
                leftFront.setPower(power / 2);
                rightBack.setPower(power / 2);
                telemetry.update();
            }
        } else {
            leftBack.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
        }
    }

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        odo.resetPosAndIMU();


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
        clawWristServo = hardwareMap.get(Servo.class, "clawWristServo");
        clawServo = hardwareMap.get(Servo.class, "clawServo");
        waitForStart();
        runtime.reset();
        odo.setOffsets(65.0, -130); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
        Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
        the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
        If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
        number of ticks per mm of your odometry pod.
         */
        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        //odo.setEncoderResolution(13.26291192);


        /*
        Set the direction that each of the two odometry pods count. The X (forward) pod should
        increase when you move the robot forward. And the Y (strafe) pod should increase when
        you move the robot to the left.
         */
        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        clawServo.setPosition(Servo.MIN_POSITION);
        odo.resetPosAndIMU();
        double angle = angleWrap(odo.getHeading());
        if (opModeIsActive()) {
            clawServo.setPosition(Servo.MIN_POSITION);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            odo.bulkUpdate();
            sleep(1000);
            LINEAR_SLIDE_DRIVE(8f, 0.9);
////            driveDistanceForward(0.6, -500, pos.getPosition().x);
//            driveForward(0.2, angleWrap(odo.getHeading()));
            while (true) { // go forward for the initial hang
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveForwardCorrection(angle, 0.4, 500, pos.getPosition().x);
                if (pos.getPosition().x >= 560) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            LINEAR_SLIDE_DRIVE(3f, -0.7);
            clawServo.setPosition(0.5);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(Servo.MAX_POSITION);
            while (true) { // back up from the hang
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.2, 590, pos.getPosition().x);
                if (pos.getPosition().x <= 559) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            clawServo.setPosition(Servo.MIN_POSITION);
            LINEAR_SLIDE_DRIVE(5f, -1);
            telemetry.update();
            while (true) { // strafe to the first block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                strafeRight(-0.6);
                if (pos.getPosition().y <= -870) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            while (true) { // angle correction
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if(angle > 0) {
                    if (angle < 2) {
                        break;
                    }
                    turnRight(0.2);
                    if (angle < 2) {
                        break;
                    }
                } else if (angle < 0 ) {
                    if (angle > -2){
                        break;
                    }
                    turnLeft(0.2);
                    if(angle > -2){
                        break;
                    }
                } else {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            clawServo.setPosition(Servo.MAX_POSITION);
            while (true) { // getting block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveForwardCorrection(angle, 0.3, 500, pos.getPosition().x);
                if (pos.getPosition().x >= 740) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            clawServo.setPosition(Servo.MIN_POSITION);
            sleep(500);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            while (true) { // back up from getting block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.5, 590, pos.getPosition().x);
                if (pos.getPosition().x <= 580) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            while (true) { // turn the 180 degrees
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (angle > 0){

                    if (angle > 173) {
                        break;
                    }
                    turnLeft(0.4);
                    if (angle > 173) {
                        break;
                    }

                } else if (angle < 0) {
                    if (angle < -173) {
                        break;
                    }
                    turnRight(0.4);
                    if (angle < -173) {
                        break;
                    }
                } else {
                    break;
                }
            }
            stopRobot();
            telemetry.update();
            while (true) { // strafe to basket
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                strafeRight(0.5);
                if (pos.getPosition().y <= -1210) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            while (true) { // angle correction
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if(angle > 0) {
                    if (angle > 178) {
                        break;
                    }
                    turnLeft(0.2);
                    if (angle > 178) {
                        break;
                    }
                } else if (angle < 0 ) {
                    if (angle < -178){
                        break;
                    }
                    turnRight(0.2);
                    if(angle < -178){
                        break;
                    }
                } else {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            LINEAR_SLIDE_DRIVE(9f, 1.0);

            stopRobot();
            telemetry.update();
            while (true) { // forward to basket
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveForwardCorrection(angle, 0.5, 500, pos.getPosition().x);
                if (pos.getPosition().x <= 30) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            sleep(1000);
            clawServo.setPosition(Servo.MAX_POSITION);
            sleep(1000);
            while (true) { // back up from basket
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.5, 590, pos.getPosition().x);
                if (pos.getPosition().x >= 20) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(Servo.MIN_POSITION);
            LINEAR_SLIDE_DRIVE(9f, -1);
            stopRobot();
            while (true) { // strafe around first block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                strafeRight(-0.5);
                if (pos.getPosition().y >= -880) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            clawServo.setPosition(Servo.MAX_POSITION);
            while (true) { // back up from basket // either move forward after or decrease go backwards.
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveBackwardCorrection(angle, -0.6, 590, pos.getPosition().x);
                if (pos.getPosition().x >= 750) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            clawWristServo.setPosition(Servo.MIN_POSITION);
            while (true) { // strafe over the second block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                strafeRight(0.4);
                if (pos.getPosition().y <= -1090) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            clawServo.setPosition(Servo.MIN_POSITION);
            sleep(500);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            while (true) { // go towards the basket // need to make shorter
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveForwardCorrection(angle, 0.6, 590, pos.getPosition().x);
                if (pos.getPosition().x <= -20) {
                    break;
                }
                telemetry.update();
            }
            telemetry.update();
            stopRobot();
            LINEAR_SLIDE_DRIVE(9f, 1.0);
            while (true) { // strafe to basket
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                strafeRight(0.4);
                if (pos.getPosition().y <= -1220) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            clawServo.setPosition(Servo.MAX_POSITION);
            sleep(99999999);

        }

    }
}