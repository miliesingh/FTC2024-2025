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

        telemetry.addData("turn", turn);
        telemetry.update();
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

    public void driveForwardXIncrease(double power, int newPos) {
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

    public void driveBackwardXDecrease(double power, int newPos) {
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

    public void strafeRightYIncrease(double power, int newPos) {
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

    public void strafeLeftYDecrease(double power, int newPos){
        double angle;
        while (true) { // strafe over to the hanging thing
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (pos.getPosition().y <= newPos) {
                break;
            }
            strafeRight(-power);
            if (pos.getPosition().y <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }

    public void angleCorrectionFacingZeroRight(double power) {
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

    public void angleCorrectionFacingZeroBothSides(double power){
        double angle;
        while (true) { // angle correction
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle > 0) {

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
    }

    public void turnAroundRightZeroTo180(double power) {
        double angle;
        while (true) { // turn the 180 degrees
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle < -172) {
                break;
            }
            turnRight(power);
            if (angle < -172) {
                break;
            }
            telemetry.update();
        }


        stopRobot();
    }

    public void turnAroundLogic180ToZero(double power) {
        double angle;
        while (true) { // turn the 180 degrees
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle > 0) {

                if (angle < 7) {
                    break;
                }
                turnRight(power);
                if (angle < 7) {
                    break;
                }

            } else if (angle < 0) {
                if (angle > -7) {
                    break;
                }
                turnLeft(power);
                if (angle > -7) {
                    break;
                }
            } else {
                break;
            }
        }
        telemetry.update();
        stopRobot();
    }

    public void driveForwardXDecrease(double power, int newPos) {
        double angle;
        while (true) { // getting the block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, power, 400, pos.getPosition().x);
            if (pos.getPosition().x <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
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
            LINEAR_SLIDE_DRIVE(8f, 0.9);
            driveForwardXIncrease(0.4, 560); // going up to hang the specimen
            LINEAR_SLIDE_DRIVE(3f, -0.7); // hanging the specimen
            clawServo.setPosition(0.5);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(Servo.MAX_POSITION);
            driveBackwardXDecrease(0.2, 559); // backing up from hanging
            clawServo.setPosition(Servo.MIN_POSITION);
            LINEAR_SLIDE_DRIVE(5f, -1);
            telemetry.update();
            sleep(333);
            strafeRightYIncrease(0.6, 690); // strafing right to the first block
            angleCorrectionFacingZeroRight(0.2);
            driveForwardXIncrease(0.6, 1100); // driving over the first block
            strafeRightYIncrease(0.6, 950); // strafing over the first block
            driveBackwardXDecrease(0.7, 150); // pushing the first block into the human player zone
            driveForwardXIncrease(0.4, 420); // going out of the human player zone
            turnAroundRightZeroTo180(0.6); // turning around
            sleep(500);
            telemetry.update();
            clawServo.setPosition(1); // opening the claw
            sleep(1000);
            driveForwardXDecrease(0.4, -150); // grabbing the block
            clawServo.setPosition(0.0); // grabbing block
            sleep(500);
            clawWristServo.setPosition(1.0);// putting the thing up
            turnAroundLogic180ToZero(0.6);
            strafeLeftYDecrease(0.8, -100); // going back over to hang the block
            angleCorrectionFacingZeroBothSides(0.2);
            LINEAR_SLIDE_DRIVE(8f, 1);
            driveForwardXIncrease(0.4, 440); // going forward to hang the block
            LINEAR_SLIDE_DRIVE(3f, -0.7);
            clawServo.setPosition(0.5);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(Servo.MAX_POSITION);
            driveBackwardXDecrease(0.2, 439); // backward after hanging the block
            stopRobot();
            clawServo.setPosition(Servo.MIN_POSITION);
            LINEAR_SLIDE_DRIVE(5f, -1);
            telemetry.update();
            sleep(9999999);
        }


    }
}