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

// This is for the sample side - puts 2 samples into the bottom basket
@Autonomous(name="newBasketSideFunction(ONLY IF THE OTHER TEAM CAN DO 5)", group="Auto2024")
public class newSampleSide extends LinearOpMode {
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
    // functions for moving the slides up
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

    // makes the angles convert from radians to regular
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
    //just a function to turn left - sets power and direction

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
    // just a function to strafe right - sets direction and power
    public void strafeRight(double power) {
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
    }

    public void strafeRightAndDiagonalUp(double power) {
        leftBack.setPower(power/2);
        rightFront.setPower(-power/2);
        leftFront.setPower(-power);
        rightBack.setPower(power);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);

    }

    public void strafeRightAndDiagonalDown(double power) {
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power/2);
        rightBack.setPower(power/2);
    }

    // function so I don't have to stop moving forward while turning

    public void softTurnLeft(double power) {
        leftBack.setPower(power / 2);
        rightFront.setPower(power);
        leftFront.setPower(power / 2);
        rightBack.setPower(power);
    }
    // function so I don't have to stop moving forward while turning
    public void softTurnRight(double power) {
        rightBack.setPower(power / 2);
        leftFront.setPower(power);
        rightFront.setPower(power / 2);
        leftBack.setPower(power);
    }
    // function to turn right - sets power and direction
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
    // function to stop the robot
    public void stopRobot() {
        leftBack.setPower(0);
        rightFront.setPower(0);
        leftFront.setPower(0);
        rightBack.setPower(0);
    }
    // function to drive forward - just sets power
    public void driveForward(double power) {
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
    }
    // function to drive backward - direction and power
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
    // function to drive forward - sets power and direction

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
    // function to turn a set number of degrees
    public void turnDegrees(double nAngle, double power) {
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // sets object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            double angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            turnRight(power);
            if (Math.abs(angle) > nAngle) {
                break;
                // if the absolute value of the angle is greater than the new one - stop
            }
            telemetry.update();
        }
        stopRobot();
    }
    // drive forward and slow down to half speed when at half the distance
    public void driveDistanceForward(double power, int distance, double xPlace) {
        // setting direction and power
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setPower(power);
        rightFront.setPower(power);
        leftFront.setPower(power);
        rightBack.setPower(power);
        // if the distance is still greater, then go at the same speed
        if (xPlace > distance) {
            if (xPlace > distance / 2) {
                leftBack.setPower(power);
                rightFront.setPower(power);
                leftFront.setPower(power);
                rightBack.setPower(power);
                telemetry.update();
            }
            // if its less then half, then go at half speed
            if (xPlace < distance / 2) {
                leftBack.setPower(power / 2);
                rightFront.setPower(power / 2);
                leftFront.setPower(power / 2);
                rightBack.setPower(power / 2);
                telemetry.update();
            }
        } else {
            // emergency stop
            leftBack.setPower(0);
            rightFront.setPower(0);
            leftFront.setPower(0);
            rightBack.setPower(0);
        }
    }
    // bunch of functions for driving distances
    public void driveForwardXIncrease(double power, int newPos) {
        double angle;
        while (true) { // go forward for the initial hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // setting the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, power, 500, pos.getPosition().x);
            // driving forward until get to the new distance
            if (pos.getPosition().x >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // drive forward for sideways
    public void driveForwardYIncrease(double power, int newPos) {
        double angle;
        while (true) { // go forward for the initial hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // setting the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, power, 500, pos.getPosition().x);
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // driving forward in the other direction sideways
    public void driveForwardYDecrease(double power, int newPos) {
        double angle;
        while (true) { // go forward for the initial hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            //setting the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // making the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, power, 500, pos.getPosition().x);
            if (pos.getPosition().y <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // driving forward in the backwards direction
    public void driveForwardXDecrease(double power, int newPos){
        double angle;
        while (true) { // forward to basket
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // setting the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // creating the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveForwardCorrection(angle, power, 500, pos.getPosition().x);
            // driving forward until the robot is less than where it is supposed to be
            if (pos.getPosition().x <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // driving backwards function
    public void driveBackwardXDecrease(double power, int newPos) {
        double angle;
        while (true) {
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // created the new object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // updating the telemetry
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
    // driving backwards in the sideways
    public void driveBackwardYDecrease(double power, int newPos) {
        double angle;
        while (true) { // back up from the hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // setting the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // creating the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveBackwardCorrection(angle, -power, 590, pos.getPosition().x);
            // driving backwards until the targeted position is hit
            if (pos.getPosition().y <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // function for driving backwards with a y increase
    public void driveBackwardYIncrease(double power, int newPos) {
        double angle;
        while (true) { // back up from the hang
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveBackwardCorrection(angle, -power, 590, pos.getPosition().x);
            //driving the robot into its position
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    //driving backwards where the x is increasing
    public void driveBackwardXIncrease(double power, int newPos){
        double angle;
        while (true) { // back up from basket
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            driveBackwardCorrection(angle, -power, 590, pos.getPosition().x);
            // driving into the position
            if (pos.getPosition().x >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // function for strafing left
    public void strafeLeftYDecrease(double power, int newPos){
        double angle;
        while (true) { // strafe over to the hanging thing
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (pos.getPosition().y <= newPos) {
                break;
            }
            // strafing until the condition is met
            strafeRight(-power);
            if (pos.getPosition().y <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // function for strafing left sideways
    public void strafeLeftXDecrease(double power, int newPos){
        double angle;
        while (true) { // strafe over to the hanging thing
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (pos.getPosition().x <= newPos) {
                break;
            }
            // strafing until the condition is met
            strafeRight(-power);
            if (pos.getPosition().x <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // strafing left sideways the other way
    public void strafeLeftXIncrease(double power, int newPos){
        double angle;
        while (true) { // strafe over to the hanging thing
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (pos.getPosition().x >= newPos) {
                break;
            }
            // strafing until the condition is met
            strafeRight(-power);
            if (pos.getPosition().x >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // angle correction code for facing zero

    public void angleCorrectionFacingZeroBothSides(double power){
        double angle;
        while (true) {
            // angle correction function
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // create a new object for the while loop because the odometry doesn't update outside of the while loop
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            // the odometry data to put on the driver hub
            telemetry.addData("Position", data);
            // creation of the new object inside the while loop
            angle = (angleWrap(odo.getHeading()));
            // if the angle is greater than 180, it goes to negatives - like 350 is coverted to -10
            odo.bulkUpdate();
            if(angle > 0) {
                // note - to the left is positive
                if (angle < 2) {
                    break;
                }
                // the reason I have two is because the program needs to stop exactly
                turnRight(power);
                //if it's left - turn right to correct
                if (angle < 2) {
                    break;
                }
            } else if (angle <= 0 ) {
                // to the right is negative
                if (angle > -2){
                    break;
                }
                turnLeft(power);
                //make the robot correct itself
                if(angle > -2){
                    break;
                }
            } else {
                break;
            }
                /* the reason everything is 2 off from 0 is because I need to account
                        for the momentum of the robot

                 */
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // angle correction code for facing 180
    public void angleCorrectionLogicFacing180BothSides(double power){
        double angle;
        while (true) { // angle correction
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if(angle > 0) {
                // if its facing right, turn left
                if (angle > 178) {
                    break;
                }
                turnLeft(power);
                if (angle > 178) {
                    break;
                }
            } else if (angle < 0 ) {
                // if its facing left turn right
                if (angle < -178){
                    break;
                }
                turnRight(power);
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
    }
    // this is for angle correction to 90 degrees
    public void angleCorrectionLogicFacingneg90BothSides(double power){
        double angle;
        while (true) { // angle correction
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if(angle < -90) {
                // if its facing right turn left
                if (angle > -89) {
                    break;
                }
                turnLeft(power);
                if (angle > -89) {
                    break;
                }
            } else if (angle > -90 ) {
                // if its facing left turn right
                if (angle < -91){
                    break;
                }
                turnRight(power);
                if(angle < 91){
                    break;
                }
            } else {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // this is another facing 180 - but its designed for more power
    public void turnAroundLogicZeroTo180(double power){
        double angle;
        while (true) { // turn the 180 degrees
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            if (angle > 0){
                // facing right turn left

                if (angle > 173) {
                    break;
                }
                turnLeft(power);
                if (angle > 173) {
                    break;
                }

            } else if (angle < 0) {
                // facing left turn right
                if (angle < -173) {
                    break;
                }
                turnRight(power);
                if (angle < -173) {
                    break;
                }
            } else {
                break;
            }
        }
        stopRobot();
        telemetry.update();
    }
    // strafing right while turned around
    public void strafeRightYDecrease(double power, int newPos){
        double angle;
        while (true) { // strafe to basket
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            strafeRight(power);
            // strafing until reaches the new and correct position
            if (pos.getPosition().y <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // strafing left while turned around
    public void strafeLeftYIncrease(double power, int newPos){
        double angle;
        while (true) { // strafe around first block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            strafeRight(-power);
            // strafing until reaches position
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // strafing right while turned around
    public void strafeRightYIncrease(double power, int newPos){
        double angle;
        while (true) { // strafe around first block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            strafeRight(power);
            // strafing until reaches position
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }

    public void strafeRightYIncreaseAndDiagonalUp(double power, int newPos){
        double angle;
        while (true) { // strafe around first block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            strafeRightAndDiagonalUp(power);
            // strafing until reaches position
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    public void strafeRightYIncreaseAndDiagonalDown(double power, int newPos){
        double angle;
        while (true) { // strafe around first block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            strafeRightAndDiagonalDown(power);
            // strafing until reaches position
            if (pos.getPosition().y >= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // strafing right while sideways
    public void strafeRightXDecrease(double power, int newPos){
        double angle;
        while (true) { // strafe around first block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            //setting the telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            strafeRight(power);
            // strafing until reaches position
            if (pos.getPosition().x <= newPos) {
                break;
            }
            telemetry.update();
        }
        stopRobot();
        telemetry.update();
    }
    // strafing right while sideways - but the other way
    public void strafeRightXIncrease(double power, int newPos){
        double angle;
        while (true) { // strafe around first block
            Pose3D pos = new Pose3D(odo.getPosition().getPosition(), odo.getVelocity().getOrientation());
            // creating the object
            String data = String.format(Locale.US, "{X: %.3f, Y: %.3f, H: %.3f}", pos.getPosition().x, pos.getPosition().y, (angleWrap(odo.getHeading())));
            telemetry.addData("Position", data);
            // setting telemetry
            angle = (angleWrap(odo.getHeading()));
            odo.bulkUpdate();
            strafeRight(power);
            // strafing until reaches position
            if (pos.getPosition().x >= newPos) {
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
            clawWristServo.setPosition(Servo.MAX_POSITION);
            odo.bulkUpdate();
            sleep(1000);
            strafeRightYIncrease(0.3, 160);// away from the wall
            driveForwardXIncrease(0.3, 340);// driving to the basket
            LINEAR_SLIDE_DRIVE(8f, 0.9);
            turnDegrees(45, -0.3); // turning to put the block into the basket
            clawServo.setPosition(Servo.MAX_POSITION);
            sleep(333);
            angleCorrectionFacingZeroBothSides(0.3);// going back to zero
            LINEAR_SLIDE_DRIVE(8f, -0.9);// linear slides down
            telemetry.update();
            angleCorrectionFacingZeroBothSides(0.3);
            clawServo.setPosition(Servo.MIN_POSITION);
            clawWristServo.setPosition(Servo.MIN_POSITION);
            turnDegrees(88, 0.3);
            strafeRightXIncrease(0.3, 162);
            angleCorrectionLogicFacingneg90BothSides(0.2);// lining up with the first block
            clawServo.setPosition(Servo.MAX_POSITION);
            driveForwardYDecrease(0.3, -550);// driving to get the first block
            clawServo.setPosition(Servo.MIN_POSITION);
            sleep(500);
            clawWristServo.setPosition(Servo.MAX_POSITION);
            driveBackwardYIncrease(0.3, -90); // has the first block
            LINEAR_SLIDE_DRIVE(8f, 0.7);
            angleCorrectionFacingZeroBothSides(0.3);// turning back
            turnDegrees(35, -0.3); //turning to put the block into the basket
            clawServo.setPosition(Servo.MAX_POSITION);
            sleep(500);
            angleCorrectionFacingZeroBothSides(0.3); // turning back around
            clawServo.setPosition(Servo.MIN_POSITION);
            LINEAR_SLIDE_DRIVE(6f, -0.9);// just so i can see the telemetry
            strafeRight(0.8);
            sleep(300);
            strafeRightAndDiagonalDown(1);
            telemetry.update();
            sleep(2700);
            stopRobot();

        }

    }
}