package org.firstinspires.ftc.team22012.universal.subsystems;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.configuration.annotations.ServoType;

public class ArmSubsystem {
    private final DcMotor linearSlideMotor, linearSlideMotor2;
    private final Servo armServo1, armServo2, armServo3;
    private final double TICKS_PER_INCH = 3163d / 38.4d;

    //max is 3163 use ~3100; min is 0
    public ArmSubsystem(DcMotor linearSlideMotor,DcMotor linearSlideMotor2, Servo armServo1, Servo armServo2, Servo armServo3) {
        this.linearSlideMotor = linearSlideMotor;
        this.linearSlideMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        this.linearSlideMotor.setTargetPosition(0);
        this.linearSlideMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.linearSlideMotor2 = linearSlideMotor2;
        this.linearSlideMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
        this.linearSlideMotor2.setTargetPosition(0);
        this.linearSlideMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.armServo1 = armServo1;
        this.armServo1.setPosition(0);
        this.armServo1.setDirection(Servo.Direction.FORWARD);

        this.armServo2 = armServo2;
        this.armServo2.setPosition(0);
        this.armServo2.setDirection(Servo.Direction.FORWARD);

        this.armServo3 = armServo3;
        this.armServo3.setPosition(0);
        this.armServo3.setDirection(Servo.Direction.REVERSE);

        this.armServo1.getController().pwmEnable();
        this.armServo2.getController().pwmEnable();
        this.armServo3.getController().pwmEnable();
    }

    public void runToPos(double inches) {
        double position = inches * TICKS_PER_INCH;
        this.linearSlideMotor.setTargetPosition((int) position);
//        this.linearSlideMotor2.setTargetPosition((int) position);
        if (position > this.linearSlideMotor.getCurrentPosition() && position > this.linearSlideMotor2.getCurrentPosition()) {
            this.moveUp();
        }
        else {
            this.moveDown();
        }
        this.linearSlideMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        this.linearSlideMotor2.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveUp(){
        linearSlideMotor.setPower(1);
        linearSlideMotor2.setPower(1);
    }
    public void moveDown(){
        linearSlideMotor.setPower(-1);
        linearSlideMotor2.setPower(-1);
    }
    public void stop() {
        this.linearSlideMotor.setPower(0);
        this.linearSlideMotor2.setPower(0);
    }
    public void runToLowJunction() {
        runToPos(12);
    }
    public void runToMidJunction() {
        runToPos(21);
    }
    public void runToHighJunction() {
        runToPos(32);
    }
    public double getHeight() {
        return linearSlideMotor.getCurrentPosition() / TICKS_PER_INCH;
    }
    public void rotateArm(double angle) {
        if (angle*5 + armServo1.getPosition()<=180.0 && angle*5 + armServo1.getPosition()>=0.0) {
            armServo1.setPosition(armServo1.getPosition() + 5 * angle);
        } else if (angle + armServo1.getPosition()<=180.0 && angle + armServo1.getPosition()>=0.0) {
            armServo1.setPosition(armServo1.getPosition() + angle);
        }
    }

    public void moveServo1(double angle) {
        armServo1.setPosition(angle);
    }
    public double getServo1Pos() {
        return armServo1.getPosition();
    }
    public void moveServo2(double angle) {

        armServo2.setPosition(angle);
    }
    public void moveServo3(double angle) {
        armServo3.setPosition(angle);
    }

    public void extendArm(double angle) {
        if (angle*5 + armServo2.getPosition()<=180.0 && angle*5 + armServo2.getPosition()>=0.0) {
            armServo2.setPosition(armServo2.getPosition() + 5 * angle);
        } else if (angle + armServo2.getPosition()<=180.0 && angle + armServo2.getPosition()>=0.0) {
            armServo2.setPosition(armServo2.getPosition() + angle);
        }
        if (armServo2.getPosition()<=90.0) {
            this.armServo3.setDirection(Servo.Direction.REVERSE);
            armServo3.setPosition(90+armServo2.getPosition());
        }
        if (armServo2.getPosition()>90.0) {
            this.armServo3.setDirection(Servo.Direction.FORWARD);
        armServo3.setPosition(90-armServo2.getPosition());
        }
    }
}