package physics;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.BodyDef.BodyType;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.Joint;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJoint;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;
import com.badlogic.gdx.physics.box2d.joints.WeldJointDef;

public class PhysicalPlayer {
	private static final float LOOSE_TORQUE = 0.2f;
	private static final float TIGHT_TORQUE = 2f;
	private Body[] touchableBodies = new Body[5];
	private Body torso;
	private Body leftUpperArm;
	private Body leftLowerArm;
	private Body leftHand;
	private Body rightUpperArm;
	private Body rightLowerArm;
	private Body rightHand;
	private Body leftUpperLeg;
	private Body leftLowerLeg;
	private Body leftFoot;
	private Body rightUpperLeg;
	private Body rightLowerLeg;
	private Body rightFoot;
	private Joint leftShoulder;
	private Joint rightShoulder;
	private Joint leftElbow;
	private Joint rightElbow;
	private Joint leftHip;
	private Joint rightHip;
	private Joint leftKnee;
	private Joint rightKnee;

	public PhysicalPlayer(Simulator simulator) {
		// TODO: this is kind of ugly
		float scale = 1.5f;
		// Torso
		BodyDef bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f, 1.25f * scale);
		torso = simulator.getSimulation().createBody(bodyDef);
		PolygonShape shape = new PolygonShape();
		shape.setAsBox(0.2f * scale, 0.25f * scale);
		FixtureDef fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		torso.createFixture(fixtureDef);
		simulator.createFixation(torso);

		// Left upper arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f - 0.2f, 1.3f * scale);
		leftUpperArm = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.075f * scale, 0.2f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		leftUpperArm.createFixture(fixtureDef);

		RevoluteJointDef jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, leftUpperArm, new Vector2(
				Simulator.WIDTH / 2f - 0.2f, 1.5f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (-0.05f * Math.PI);
		jointDef.lowerAngle = (float) (-0.9f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		leftShoulder = simulator.getSimulation().createJoint(jointDef);

		// Left lower arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f - 0.2f, 0.95f * scale);
		leftLowerArm = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f * scale, 0.15f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		leftLowerArm.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(leftUpperArm, leftLowerArm, new Vector2(
				Simulator.WIDTH / 2f - 0.2f, 1.10f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (-0.05f * Math.PI);
		jointDef.lowerAngle = (float) (-0.85f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		leftElbow = simulator.getSimulation().createJoint(jointDef);

		// Left hand
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f - 0.2f, 0.75f * scale);
		leftHand = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f * scale, 0.05f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x4;
		fixtureDef.filter.maskBits = 0x5;
		leftHand.createFixture(fixtureDef);

		WeldJointDef wJointDef = new WeldJointDef();
		wJointDef.initialize(leftLowerArm, leftHand, new Vector2(
				Simulator.WIDTH / 2f - 0.2f, 0.8f * scale));
		wJointDef.collideConnected = false;
		// jointDef.enableLimit = true;
		// jointDef.upperAngle = (float) (0.05f * Math.PI);
		// jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		// jointDef.enableMotor = true;
		// jointDef.motorSpeed = 0;
		// jointDef.maxMotorTorque = 2f;
		simulator.getSimulation().createJoint(wJointDef);

		// Right upper arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f + 0.2f, 1.3f * scale);
		rightUpperArm = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.075f * scale, 0.2f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		rightUpperArm.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, rightUpperArm, new Vector2(
				Simulator.WIDTH / 2f + 0.2f, 1.5f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.9f * Math.PI);
		jointDef.lowerAngle = (float) (0.05f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		rightShoulder = simulator.getSimulation().createJoint(jointDef);

		// Right lower arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f + 0.2f, 0.95f * scale);
		rightLowerArm = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f * scale, 0.15f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		rightLowerArm.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(rightUpperArm, rightLowerArm, new Vector2(
				Simulator.WIDTH / 2f + 0.2f, 1.10f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.85f * Math.PI);
		jointDef.lowerAngle = (float) (0.05f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		rightElbow = simulator.getSimulation().createJoint(jointDef);

		// Right hand
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f + 0.2f, 0.75f * scale);
		rightHand = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f * scale, 0.05f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x4;
		fixtureDef.filter.maskBits = 0x5;
		rightHand.createFixture(fixtureDef);

		wJointDef = new WeldJointDef();
		wJointDef.initialize(rightLowerArm, rightHand, new Vector2(
				Simulator.WIDTH / 2f + 0.2f, 0.8f * scale));
		wJointDef.collideConnected = false;
		// jointDef.enableLimit = true;
		// jointDef.upperAngle = (float) (0.05f * Math.PI);
		// jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		// jointDef.enableMotor = true;
		// jointDef.motorSpeed = 0;
		// jointDef.maxMotorTorque = 2f;
		simulator.getSimulation().createJoint(wJointDef);

		// Left upper leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f - 0.1f, 0.75f * scale);
		leftUpperLeg = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.085f * scale, 0.25f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		leftUpperLeg.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, leftUpperLeg, new Vector2(
				Simulator.WIDTH / 2f - 0.1f, 1f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (-0.05f * Math.PI);
		jointDef.lowerAngle = (float) (-0.75f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		leftHip = simulator.getSimulation().createJoint(jointDef);

		// Left lower leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f - 0.1f, 0.275f * scale);
		leftLowerLeg = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f * scale, 0.225f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		leftLowerLeg.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(leftUpperLeg, leftLowerLeg, new Vector2(
				Simulator.WIDTH / 2f - 0.1f, 0.5f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (0.05f * Math.PI);
		jointDef.upperAngle = (float) (0.85f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		leftKnee = simulator.getSimulation().createJoint(jointDef);

		// Left foot
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f - 0.1f, 0.05f * scale);
		leftFoot = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f * scale, 0.05f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x8;
		fixtureDef.filter.maskBits = 0x9;
		leftFoot.createFixture(fixtureDef);

		wJointDef = new WeldJointDef();
		wJointDef.initialize(leftLowerLeg, leftFoot, new Vector2(
				Simulator.WIDTH / 2f - 0.1f, 0.1f * scale));
		wJointDef.collideConnected = false;
		// jointDef.enableLimit = true;
		// jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		// jointDef.upperAngle = (float) (0.05f * Math.PI);
		// jointDef.enableMotor = true;
		// jointDef.motorSpeed = 0;
		// jointDef.maxMotorTorque = 2f;
		simulator.getSimulation().createJoint(wJointDef);

		// simulator.createFixation(lFoot);

		// Right upper leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f + 0.1f, 0.75f * scale);
		rightUpperLeg = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.085f * scale, 0.25f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		rightUpperLeg.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, rightUpperLeg, new Vector2(
				Simulator.WIDTH / 2f + 0.1f, 1f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.75f * Math.PI);
		jointDef.lowerAngle = (float) (0.05f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		rightHip = simulator.getSimulation().createJoint(jointDef);

		// Right lower leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f + 0.1f, 0.275f * scale);
		rightLowerLeg = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f * scale, 0.225f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		rightLowerLeg.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(rightUpperLeg, rightLowerLeg, new Vector2(
				Simulator.WIDTH / 2f + 0.1f, 0.5f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (-0.85f * Math.PI);
		jointDef.upperAngle = (float) (-0.05f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		rightKnee = simulator.getSimulation().createJoint(jointDef);

		// Right foot
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(Simulator.WIDTH / 2f + 0.1f, 0.05f * scale);
		rightFoot = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f * scale, 0.05f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x8;
		fixtureDef.filter.maskBits = 0x9;
		rightFoot.createFixture(fixtureDef);

		wJointDef = new WeldJointDef();
		wJointDef.initialize(rightLowerLeg, rightFoot, new Vector2(
				Simulator.WIDTH / 2f + 0.1f, 0.1f * scale));
		wJointDef.collideConnected = false;
		// jointDef.enableLimit = true;
		// jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		// jointDef.upperAngle = (float) (0.05f * Math.PI);
		// jointDef.enableMotor = true;
		// jointDef.motorSpeed = 0;
		// jointDef.maxMotorTorque = 2f;
		simulator.getSimulation().createJoint(wJointDef);

		// simulator.createFixation(rFoot);

		touchableBodies[0] = leftHand;
		touchableBodies[1] = rightHand;
		touchableBodies[2] = leftFoot;
		touchableBodies[3] = rightFoot;
		touchableBodies[4] = torso;
	}

	public void loosenLimb(Body touchedBody) {
		changeLimbTorque(touchedBody, LOOSE_TORQUE);
	}

	public void tightenLimb(Body touchedBody) {
		changeLimbTorque(touchedBody, TIGHT_TORQUE);
	}

	private void changeLimbTorque(Body touchedBody, float torque) {
		if (touchedBody == leftHand) {
			changeJointTorque(leftShoulder, torque);
			changeJointTorque(leftElbow, torque);
		}
		if (touchedBody == rightHand) {
			changeJointTorque(rightShoulder, torque);
			changeJointTorque(rightElbow, torque);
		}
		if (touchedBody == leftFoot) {
			changeJointTorque(leftHip, torque);
			changeJointTorque(leftKnee, torque);
		}
		if (touchedBody == rightFoot) {
			changeJointTorque(rightHip, torque);
			changeJointTorque(rightKnee, torque);
		}
	}

	public void changeJointTorque(Joint joint, float torque) {
		((RevoluteJoint) joint).setMaxMotorTorque(torque);
	}

	public boolean isHand(Body touchedBody) {
		return touchedBody == leftHand || touchedBody == rightHand;
	}

	public boolean isFoot(Body touchedBody) {
		return touchedBody == leftFoot || touchedBody == rightFoot;
	}

	public Body[] getTouchableBodies() {
		return touchableBodies;
	}

	public Body getTorso() {
		return torso;
	}

	public Body getLeftUpperArm() {
		return leftUpperArm;
	}

	public Body getLeftLowerArm() {
		return leftLowerArm;
	}

	public Body getLeftHand() {
		return leftHand;
	}

	public Body getRightUpperArm() {
		return rightUpperArm;
	}

	public Body getRightLowerArm() {
		return rightLowerArm;
	}

	public Body getRightHand() {
		return rightHand;
	}

	public Body getLeftUpperLeg() {
		return leftUpperLeg;
	}

	public Body getLeftLowerLeg() {
		return leftLowerLeg;
	}

	public Body getLeftFoot() {
		return leftFoot;
	}

	public Body getRightUpperLeg() {
		return rightUpperLeg;
	}

	public Body getRightLowerLeg() {
		return rightLowerLeg;
	}

	public Body getRightFoot() {
		return rightFoot;
	}

}
