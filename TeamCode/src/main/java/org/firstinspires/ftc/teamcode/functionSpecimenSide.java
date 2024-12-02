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
@Autonomous(name="functionSpecimenSide", group="Auto2024")
public class functionSpecimenSide extends LinearOpMode {
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

    public void driveForwardXIncrease(double power, int newPos){
        double angle;
        while (true) { // go forward for the initial hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, power, 500, pos.getPosition().x);
            if (pos.getPosition().x >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    public void driveBackwardXDecrease(double power, int newPos){
        double angle;
        while (true) { // back up from the hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveBackwardCorrection(angle, -power, 590, pos.getPosition().x);
            if (pos.getPosition().x <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    public void strafeRightYIncrease(double power, int newPos){
        double angle;
        while (true) { // strafe to the blocks place
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (pos.getPosition().y >= newPos) {
                break;
            }
            strafeRight(power);
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }

    public void angleCorrectionFacingZero(double power){
        double angle;
        while (true) { // angle correction
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle < 2) {
                break;
            }
            turnRight(power);
            if (angle < 2) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
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
            sleep(333);
            LINEAR_SLIDE_DRIVE(8f,0.9);
////            driveDistanceForward(0.6, -500, pos.getPosition().x);
//            driveForward(0.2, angleWrap(odo.getHeading()));
            driveForwardXIncrease(0.4, 560);
            LINEAR_SLIDE_DRIVE(3f, -0.7);
            clawServo.setPosition(0.5);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(Servo.MAX_POSITION);
            driveBackwardXDecrease(0.2, 559);
            clawServo.setPosition(Servo.MIN_POSITION);
            LINEAR_SLIDE_DRIVE(5f, -1);
            telemetry.update();
            sleep(333);
            strafeRightYIncrease(0.6, 690);
            angleCorrectionFacingZero(0.2);
            driveForwardXIncrease(0.6, 1100);
            strafeRightYIncrease(0.6, 950);
            driveBackwardXDecrease(0.7, 150);
            driveForwardXIncrease(0.4, 420);

            while (true) { // turn the 180 degrees
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (angle < -172) {
                    break;
                }
                turnRight(0.6);
                if (angle < -172) {
                    break;
                }
                telemetry.update();
            }


            stopRobot();
            sleep(500);
            telemetry.update();
            clawServo.setPosition(1); // open
            sleep(1000);

            while (true) { // getting the block
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                driveForwardCorrection(angle, 0.4, 400, pos.getPosition().x);
                if (pos.getPosition().x <= -150) {
                    break;
                }
                telemetry.update();
            }
            stopRobot();
            telemetry.update();
            clawServo.setPosition(0.0); // grabbing block
            sleep(500);
            clawWristServo.setPosition(1.0);// putting the thing up

            while (true) { // turn the 180 degrees
                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
                telemetry.addData("Position", data);
                angle = (angleWrap(odo.getHeading()));
                odo.bulkUpdate();
                if (angle > 0){

                    if (angle < 7) {
                        break;
                    }
                    turnRight(0.6);
                    if (angle < 7) {
                        break;
                    }

                } else if (angle < 0) {
                    if (angle > -7) {
                        break;
                    }
                    turnLeft(0.6);
                    if (angle > -7) {
                        break;
                    }
                } else {
                    break;
                }
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
        while (true) { // strafe over to the hanging thing
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (pos.getPosition().y <= -100) {
                break;
            }
            strafeRight(-0.8);
            if (pos.getPosition().y <= -100) {
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
            if (angle > 0){

                if (angle < 2) {
                    break;
                }
                turnRight(0.2);
                if (angle < 2) {
                    break;
                }

            } else if (angle < 0) {
                if (angle > -2) {
                    break;
                }
                turnLeft(0.2);
                if (angle > -2) {
                    break;
                }
            } else {
                break;
            }
        }
        stopRobot();
        telemetry.update();
        LINEAR_SLIDE_DRIVE(8f,1);
        ////            driveDistanceForward(0.6, -500, pos.getPosition().x);
        //            driveForward(0.2, angleWrap(odo.getHeading()));
        while (true) { // go forward for the 2nd  hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, 0.4, 500, pos.getPosition().x);
            if (pos.getPosition().x >= 440) {
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
            if (pos.getPosition().x <= 439) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        clawServo.setPosition(Servo.MIN_POSITION);
        LINEAR_SLIDE_DRIVE(5f, -1);
        telemetry.update();
        sleep(9999999);
//            stopRobot();
//            telemetry.update();
//            stopRobot();
//            sleep(333);
//            while (true) { // push 1st block into the zone
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                driveBackwardCorrection(angle, -0.6, -200, pos.getPosition().x);
//                if (pos.getPosition().x <= -200) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            odo.resetPosAndIMU();
//            sleep(333);
//            while (true) { // go up for the second block
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                if (pos.getPosition().x >= 870) {
//                    break;
//                }
//                driveForwardCorrection(angle, 0.6, 950, pos.getPosition().x);
//                if (pos.getPosition().x >= 870) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            sleep(500);
//            while (true) { // strafe over the 2nd block
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                if (pos.getPosition().y >= 110) {
//                    break;
//                }
//                strafeRight(0.4);
//                if (pos.getPosition().y >= 110) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            sleep(333);
//            while (true) { // push the second block into the zone
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                driveBackwardCorrection(angle, -0.6, 250, pos.getPosition().x);
//                if (pos.getPosition().x <= 150) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            odo.resetPosAndIMU();
//            stopRobot();
//            sleep(500);
//            telemetry.update();
//            while (true) { // move up a little for the human player
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                if (pos.getPosition().x >= 250) {
//                    break;
//                }
//                driveForwardCorrection(angle, 0.5, 200, pos.getPosition().x);
//                if (pos.getPosition().x >= 250) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            odo.resetPosAndIMU();
//            sleep(5000);
//            turnDegrees(10, -0.4);
//            while (true) { // turn the 180
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                if (angle < -140) {
//                    break;
//                }
//                turnLeft(0.4);
//                if (angle < -140) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            while (true) { // grab the block
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                if (pos.getPosition().x >= 200) {
//                    break;
//                }
//                driveForwardCorrection(angle, 0.5, 200, pos.getPosition().x);
//                if (pos.getPosition().x >= 200) {
//                    break;
//                }
//                telemetry.update();
//            }
//            telemetry.update();
//            stopRobot();
//            while (true) { // back up with said block
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                driveBackwardCorrection(angle, -0.4, 250, pos.getPosition().x);
//                if (pos.getPosition().x <= 100) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            telemetry.update();
//            odo.resetPosAndIMU();
//            sleep(500);
//            stopRobot();
//            while (true) { // strafe back to the hanging thing
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                if (pos.getPosition().y <= 100) {
//                    break;
//                }
//                strafeRight(0.4);
//                if (pos.getPosition().y <= 100) {
//                    break;
//                }
//                telemetry.update();
//            }
//            telemetry.update();
//            stopRobot();
//            turnDegrees(10, 0.4);
//            while (true) { // turn to face said hanging thing
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                if (angle > 0) {
//                    break;
//                }
//                turnRight(0.4);
//                if (angle > 0) {
//                    break;
//                }
//                telemetry.update();
//            }
//            telemetry.update();
//            stopRobot();
//            while (true) { // put the sample on the hanging thing
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                if (pos.getPosition().x >= 400) {
//                    break;
//                }
//                driveForwardCorrection(angle, 0.5, 400, pos.getPosition().x);
//                if (pos.getPosition().x >= 400) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
//            sleep(3000);
//            telemetry.update();
//            while (true) { // back up from said hanging thing
//                Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
//                String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
//                telemetry.addData("Position", data);
//                angle = (angleWrap(odo.getHeading()));
//                odo.bulkUpdate();
//                driveBackwardCorrection(angle, -0.4, 0, pos.getPosition().x);
//                if (pos.getPosition().x <= 0) {
//                    break;
//                }
//                telemetry.update();
//            }
//            stopRobot();
    }


}