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
@Autonomous(name="specimenSide14+", group="Auto2024")
public class specimenSide14 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFront = null;
    private DcMotor leftFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftBack = null;

    private DcMotor linSlideR = null;
    private DcMotor linSlideL = null;
    private Servo clawWristServo = null;
    private Servo clawServo = null;

    private Servo intakeWristServo = null;

    private DistanceSensor sensorRange;


    GoBildaPinpointDriver odo;

    //basic function for moving the linear slides up
    public void linSlidesUp(double power){
        linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideL.setPower(power);
        linSlideR.setPower(power);
    }
    //function for moving slides down
    public void linSlidesDown(double power){
        linSlideL.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideR.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideL.setPower(power);
        linSlideR.setPower(power);

    }
    //function to keep the slides level

    public void linSlidesStay(){
        linSlideL.setDirection(DcMotorSimple.Direction.FORWARD);
        linSlideR.setDirection(DcMotorSimple.Direction.REVERSE);
        linSlideL.setPower(0.07);
        linSlideR.setPower(0.07);
    }
    // old function for moving the slides up and down
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
    //function to convert an angle from radians to degrees
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
    // function to drive forward
    public void driveForwards(double power){
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setPower(-power);
        rightFront.setPower(-power);
        leftFront.setPower(-power);
        rightBack.setPower(-power);
    }
    // function to control the robots distance from an object

    public void controlDistance(double distance, double power){
        telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
        double theDistance = sensorRange.getDistance(DistanceUnit.CM);
        while (theDistance > distance){
            theDistance = sensorRange.getDistance(DistanceUnit.CM);
            driveForwards(power);
            telemetry.update();
        }
        stopRobot();
    }
    // function to turn left
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
    // function to strafe right

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
    // function to strafe right at a diagonal
    public void strafeRightAndDiagonal(double power) {
        leftBack.setPower(power/2);
        rightFront.setPower(-power/2);
        leftFront.setPower(-power);
        rightBack.setPower(power);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);

    }
    // function to turn left while still moving forward

    public void softTurnLeft(double power) {
        leftBack.setPower(power / 2);
        rightFront.setPower(power);
        leftFront.setPower(power / 2);
        rightBack.setPower(power);
    }
    // function to turn right while still moving forward

    public void softTurnRight(double power) {
        rightBack.setPower(power / 2);
        leftFront.setPower(power);
        rightFront.setPower(power / 2);
        leftBack.setPower(power);
    }

    // function for turning right
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
    // functino for stopping thr robot
    public void stopRobot() {
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
    }
    // yet another functino to drive forward

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
    // function to drive backward
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
    // function to drive forward

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
    // function to turn an angle
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
    // this is a function to drive forward a set number of mm forward - and slow down at halfway
    public void driveDistanceForward(double power, int distance, double xPlace) {
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        // setting stuff
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
            // slowing down at half
        } else {
            leftBack.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
        }
    }
    // driving forward for a set amount of mm

    public void driveForwardXIncrease(double power, int newPos) {
        double angle;
        while (true) { // go forward for the initial hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForward(power);
            if (pos.getPosition().x >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // this function utilizes the motor encoders to move the linear slides up while driving forward a set amount
    public void driveForwardWithLinAndSlideUp(double drivePower, int newPos, double slidePower, double inches){
        double angle;
        // making the linear slides run using the encoders
        linSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // setting the ticks
        float ticksPerInch = 450.149432158f;
        float f_ticks = (float) (ticksPerInch * inches);
        int ticks = Math.round(f_ticks);
        linSlideL.setTargetPosition(ticks);
        linSlideR.setTargetPosition(ticks);
        // setting the drive forward function

        while (true) { // go forward for the initial hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForward(drivePower);
            if((linSlideR.getCurrentPosition() >= linSlideR.getTargetPosition()) && linSlideL.getCurrentPosition() >= (linSlideL.getTargetPosition())){
                linSlidesStay();
            }
            else {
                // setting the slides to go up and stop
                linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlidesUp(slidePower);
                linSlideL.setTargetPosition(ticks);
                linSlideR.setTargetPosition(ticks);
            }
            if (pos.getPosition().x >= newPos) {
                break;
            }
            telemetry.update();
        }
        linSlidesStay();
        stopRobot();
        telemetry.update();
    }

    // driving back a certain amount of mm
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
    // strafing right a certain amount of mm
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
    // attempting to strafe right at a diagonal a certain amount
    public void strafeRightYIncreaseAndDiagonal(double power, int newPos) {
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
            strafeRightAndDiagonal(power);
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }

    // this strafes right while moving the slides down using the motor encoders
    public void strafeRightYIncreaseSlideDown(double power, int newPos, double slidePower, double slidePlace) {
        double angle;
        // setting the motors to use the encoders
        linSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // making them use ticks

        float ticksPerInch = 450.149432158f;
        float f_ticks = (float) (ticksPerInch * slidePlace);
        int ticks = Math.round(f_ticks);
        linSlideL.setTargetPosition(ticks);
        linSlideR.setTargetPosition(ticks);

        // having the odometry to strafe right a set distance
        while (true) { // strafe to the blocks place
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            telemetry.addData("Slides Current Position", linSlideR.getCurrentPosition());
            telemetry.addData("Slides Target Position", linSlideR.getTargetPosition());
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if((linSlideR.getCurrentPosition() <= linSlideR.getTargetPosition()) && linSlideL.getCurrentPosition() <= (linSlideL.getTargetPosition())){
                linSlidesStay();
            }
            else {
                // having the slides move a set distance
                linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlidesUp(-slidePower);
                linSlideL.setTargetPosition(ticks);
                linSlideR.setTargetPosition(ticks);
            }
            strafeRight(power);
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        linSlidesStay();
        stopRobot();
        telemetry.update();
    }

    // function for strafing left a certain amount of mm
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
    // strafing left while moving the slides up - this is to get to the submersible

    public void strafeLeftYDecreaseSlideUp(double power, int newPos, double slidePower, double inches){
        double angle;

        // setting the motors to using encoders

        linSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // setting the ticks - so like we can actually control them

        float ticksPerInch = 450.149432158f;
        float f_ticks = (float) (ticksPerInch * inches);
        int ticks = Math.round(f_ticks);
        linSlideL.setTargetPosition(ticks);
        linSlideR.setTargetPosition(ticks);

        // the function to strafe left a certain amout of mm

        while (true) { // strafe over to the hanging thing
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if((linSlideR.getCurrentPosition() >= linSlideR.getTargetPosition()) && linSlideL.getCurrentPosition() >= (linSlideL.getTargetPosition())){
                linSlidesStay();
            }
            else {
                // moving the slides a set amount
                linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                linSlidesUp(slidePower);
                linSlideL.setTargetPosition(ticks);
                linSlideR.setTargetPosition(ticks);
            }
            strafeRight(-power);
            if (pos.getPosition().y <= newPos) {
                break;
            }
            telemetry.update();
        }
        linSlidesStay();
        stopRobot();
        telemetry.update();
    }
    // using logic to move the angle towards 0 when facing right a bit

    public void angleCorrectionFacingZeroRight(double power) {
        double angle;
        while (true) { // angle correction
            // setting the object
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle < 2) {
                break;
            }
            // if the angle is > 2 then turn right until not
            turnRight(power);
            if (angle < 2) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
    }

    // setting to it faces zero no matter which sides they are on
    public void angleCorrectionFacingZeroBothSides(double power){
        double angle;
        while (true) { // angle correction
            // setting the objects
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle > 0) {

                if (angle < 2) {
                    break;
                }
                // if faced left turn right
                turnRight(0.2);
                if (angle < 2) {
                    break;
                }

            } else if (angle < 0) {
                if (angle > -2) {
                    break;
                }
                // if looking towards the right, turn left
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

    // this function to turn around 180 degrees - only right though
    public void turnAroundRightZeroTo180(double power) {
        double angle;
        while (true) { // turn the 180 degrees
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // setting the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle < -172) {
                break;
            }
            // turning right until the conditions are met
            turnRight(power);
            if (angle < -172) {
                break;
            }
            telemetry.update();
        }


        stopRobot();
    }

    //function to turn around no matter where you are facing
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
                // totally depends - if you are already facing right, continue
                turnRight(power);
                if (angle < 7) {
                    break;
                }

            } else if (angle < 0) {
                if (angle > -7) {
                    break;
                }
                // if already facing left, continue
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
    // this like if it has turneed around and you need to continue

    public void driveForwardXDecrease(double power, int newPos) {
        double angle;
        while (true) { // getting the block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForward(power);
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
        sensorRange = hardwareMap.get(DistanceSensor.class, "distanceSensor");
        intakeWristServo = hardwareMap.get(Servo.class, "intakeWristServo");

        waitForStart();
        runtime.reset();

        //setting encoders for the linear slides
        linSlideL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideL.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linSlideR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        linSlideR.setMode(DcMotor.RunMode.RUN_TO_POSITION);


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
        intakeWristServo.setPosition(Servo.MIN_POSITION);
        odo.resetPosAndIMU();
        double angle = angleWrap(odo.getHeading());
        if (opModeIsActive()) {
            clawServo.setPosition(Servo.MIN_POSITION);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            odo.bulkUpdate();
            sleep(333);
            driveForwardWithLinAndSlideUp(0.378, 560, 1.0, 8); // going up to hang the specimen
            telemetry.update();
            LINEAR_SLIDE_DRIVE(2f, -0.7); // hanging the specimen
            clawServo.setPosition(0.5);
            driveBackwardXDecrease(0.5, 562); // backing up from hanging
            clawServo.setPosition(Servo.MIN_POSITION);
            telemetry.update();
            sleep(333);
            strafeRightYIncreaseSlideDown(0.7, 600, 1.0, -3.4);// strafing right to the first block
            clawWristServo.setPosition(1.0);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            angleCorrectionFacingZeroBothSides(0.4);
            driveForwardWithLinAndSlideUp(0.7, 1100, 0, 0); // driving over the first block
            strafeRightYIncrease(0.7, 840); // strafing over the first block
            driveBackwardXDecrease(0.8, 310); // pushing the first block into the player zone
            driveForwardXIncrease(0.8, 440); // going out of the human player zone
            turnAroundRightZeroTo180(0.5); // turning around
            sleep(500);
            telemetry.update();
            clawServo.setPosition(1); // opening the claw
            controlDistance(46, 0.6); // grabbing the block
            stopRobot();
            clawServo.setPosition(0.0); // grabbing block
            sleep(500);
            clawWristServo.setPosition(1.0);// putting the thing up
            LINEAR_SLIDE_DRIVE(2f, 1.0);
            turnAroundLogic180ToZero(0.6);
            strafeLeftYDecreaseSlideUp(0.9, 50, 1.0, 4.5); // going back over to hang the block
            angleCorrectionFacingZeroBothSides(0.1);
            controlDistance(37, 0.5); // going forward to hang the block
            LINEAR_SLIDE_DRIVE(3f, -0.7); // hanging the specimen
            clawServo.setPosition(0.5);
            sleep(500);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(0.5);
            driveBackwardXDecrease(0.4, 559); // backing up from hanging
            clawServo.setPosition(Servo.MIN_POSITION);
            strafeRightYIncreaseSlideDown(0.8, 900, 1.0, -3.4);
            turnAroundRightZeroTo180(0.6);
            clawServo.setPosition(0.5);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            controlDistance(46, 0.6); // grabbing the block
            stopRobot();
            clawServo.setPosition(0.0); // grabbing block
            sleep(500);
            clawWristServo.setPosition(1.0);// putting the thing up
            LINEAR_SLIDE_DRIVE(2f, 1.0);
            turnAroundLogic180ToZero(0.6);
            strafeLeftYDecreaseSlideUp(0.9, 20, 1.0, 4.5); // going back over to hang the block
            angleCorrectionFacingZeroBothSides(0.1);
            controlDistance(37, 0.5); // going forward to hang the block
            LINEAR_SLIDE_DRIVE(3f, -0.7); // hanging the specimen
            clawServo.setPosition(0.5);
            sleep(500);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(0.5);
            driveBackwardXDecrease(0.3, 559); // backing up from hanging

        }


    }
}