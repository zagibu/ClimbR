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

public class PhysicalPlayer {
	WorldSimulator simulator;
	Body[] touchableBodies = new Body[4];
	Body torso;
	Body luArm;
	Body llArm;
	Body lHand;
	Body ruArm;
	Body rlArm;
	Body rHand;
	Body luLeg;
	Body llLeg;
	Body lFoot;
	Body ruLeg;
	Body rlLeg;
	Body rFoot;
	Joint lShoulder;
	Joint rShoulder;
	Joint lElbow;
	Joint rElbow;
	Joint lHip;
	Joint rHip;
	Joint lKnee;
	Joint rKnee;
	boolean climbing = true;

	public PhysicalPlayer(WorldSimulator simulator) {
		this.simulator = simulator;
		float scale = 1.5f;
		// Torso
		BodyDef bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f, 1.25f * scale);
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
		bodyDef.position.set(WorldSimulator.WIDTH / 2f - 0.2f, 1.3f * scale);
		luArm = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.075f * scale, 0.2f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		luArm.createFixture(fixtureDef);

		RevoluteJointDef jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, luArm, new Vector2(
				WorldSimulator.WIDTH / 2f - 0.2f, 1.5f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (-0.05f * Math.PI);
		jointDef.lowerAngle = (float) (-0.9f * Math.PI);
		lShoulder = simulator.getSimulation().createJoint(jointDef);

		// Left lower arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f - 0.2f, 0.95f * scale);
		llArm = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f * scale, 0.15f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		llArm.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(luArm, llArm, new Vector2(
				WorldSimulator.WIDTH / 2f - 0.2f, 1.10f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0f * Math.PI);
		jointDef.lowerAngle = (float) (-0.85f * Math.PI);
		lElbow = simulator.getSimulation().createJoint(jointDef);

		// Left hand
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f - 0.2f, 0.75f * scale);
		lHand = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f * scale, 0.05f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x4;
		fixtureDef.filter.maskBits = 0x5;
		lHand.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(llArm, lHand, new Vector2(
				WorldSimulator.WIDTH / 2f - 0.2f, 0.8f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.05f * Math.PI);
		jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		simulator.getSimulation().createJoint(jointDef);

		// Right upper arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f + 0.2f, 1.3f * scale);
		ruArm = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.075f * scale, 0.2f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		ruArm.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, ruArm, new Vector2(
				WorldSimulator.WIDTH / 2f + 0.2f, 1.5f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.9f * Math.PI);
		jointDef.lowerAngle = (float) (0.05f * Math.PI);
		rShoulder = simulator.getSimulation().createJoint(jointDef);

		// Right lower arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f + 0.2f, 0.95f * scale);
		rlArm = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f * scale, 0.15f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		rlArm.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(ruArm, rlArm, new Vector2(
				WorldSimulator.WIDTH / 2f + 0.2f, 1.10f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.85f * Math.PI);
		jointDef.lowerAngle = (float) (0f * Math.PI);
		rElbow = simulator.getSimulation().createJoint(jointDef);

		// Right hand
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f + 0.2f, 0.75f * scale);
		rHand = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f * scale, 0.05f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x4;
		fixtureDef.filter.maskBits = 0x5;
		rHand.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(rlArm, rHand, new Vector2(
				WorldSimulator.WIDTH / 2f + 0.2f, 0.8f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.05f * Math.PI);
		jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		simulator.getSimulation().createJoint(jointDef);

		// Left upper leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f - 0.1f, 0.75f * scale);
		luLeg = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.085f * scale, 0.25f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		luLeg.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, luLeg, new Vector2(
				WorldSimulator.WIDTH / 2f - 0.1f, 1f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0f * Math.PI);
		jointDef.lowerAngle = (float) (-0.75f * Math.PI);
		lHip = simulator.getSimulation().createJoint(jointDef);

		// Left lower leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f - 0.1f, 0.275f * scale);
		llLeg = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f * scale, 0.225f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		llLeg.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(luLeg, llLeg, new Vector2(
				WorldSimulator.WIDTH / 2f - 0.1f, 0.5f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (0f * Math.PI);
		jointDef.upperAngle = (float) (0.85f * Math.PI);
		lKnee = simulator.getSimulation().createJoint(jointDef);

		// Left foot
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f - 0.1f, 0.05f * scale);
		lFoot = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f * scale, 0.05f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x8;
		fixtureDef.filter.maskBits = 0x9;
		lFoot.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(llLeg, lFoot, new Vector2(
				WorldSimulator.WIDTH / 2f - 0.1f, 0.1f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		jointDef.upperAngle = (float) (0.05f * Math.PI);
		simulator.getSimulation().createJoint(jointDef);

		simulator.createFixation(lFoot);

		// Right upper leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f + 0.1f, 0.75f * scale);
		ruLeg = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.085f * scale, 0.25f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		ruLeg.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, ruLeg, new Vector2(
				WorldSimulator.WIDTH / 2f + 0.1f, 1f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.75f * Math.PI);
		jointDef.lowerAngle = (float) (0f * Math.PI);
		rHip = simulator.getSimulation().createJoint(jointDef);

		// Right lower leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f + 0.1f, 0.275f * scale);
		rlLeg = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f * scale, 0.225f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x2;
		fixtureDef.filter.maskBits = 0x1;
		rlLeg.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(ruLeg, rlLeg, new Vector2(
				WorldSimulator.WIDTH / 2f + 0.1f, 0.5f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (-0.85f * Math.PI);
		jointDef.upperAngle = (float) (0f * Math.PI);
		rKnee = simulator.getSimulation().createJoint(jointDef);

		// Right foot
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(WorldSimulator.WIDTH / 2f + 0.1f, 0.05f * scale);
		rFoot = simulator.getSimulation().createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f * scale, 0.05f * scale);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		fixtureDef.filter.categoryBits = 0x8;
		fixtureDef.filter.maskBits = 0x9;
		rFoot.createFixture(fixtureDef);

		jointDef = new RevoluteJointDef();
		jointDef.initialize(rlLeg, rFoot, new Vector2(
				WorldSimulator.WIDTH / 2f + 0.1f, 0.1f * scale));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		jointDef.upperAngle = (float) (0.05f * Math.PI);
		simulator.getSimulation().createJoint(jointDef);

		simulator.createFixation(rFoot);

		touchableBodies[0] = lHand;
		touchableBodies[1] = rHand;
		touchableBodies[2] = lFoot;
		touchableBodies[3] = rFoot;
	}

	public void toggleClimbing() {
		if (climbing) {
			stopClimbing();
			climbing = false;
		} else {
			startClimbing();
			climbing = true;
		}
	}

	public void startClimbing() {
		RevoluteJoint joint;
		float torque = 10f;
		joint = (RevoluteJoint) lHip;
		joint.setMotorSpeed(1f);
		joint.setMaxMotorTorque(torque);
		joint.enableMotor(true);
		joint = (RevoluteJoint) rHip;
		joint.setMotorSpeed(-1f);
		joint.setMaxMotorTorque(torque);
		joint.enableMotor(true);
		joint = (RevoluteJoint) lKnee;
		joint.setMotorSpeed(-1f);
		joint.setMaxMotorTorque(torque);
		joint.enableMotor(true);
		joint = (RevoluteJoint) rKnee;
		joint.setMotorSpeed(1f);
		joint.setMaxMotorTorque(torque);
		joint.enableMotor(true);
		joint = (RevoluteJoint) lShoulder;
		joint.setMotorSpeed(1f);
		joint.setMaxMotorTorque(torque);
		joint.enableMotor(true);
		joint = (RevoluteJoint) lElbow;
		joint.setMotorSpeed(-1f);
		joint.setMaxMotorTorque(torque);
		joint.enableMotor(true);
		joint = (RevoluteJoint) rShoulder;
		joint.setMotorSpeed(-1f);
		joint.setMaxMotorTorque(torque);
		joint.enableMotor(true);
		joint = (RevoluteJoint) rElbow;
		joint.setMotorSpeed(1f);
		joint.setMaxMotorTorque(torque);
		joint.enableMotor(true);
		if (torso.getUserData() != null) {
			simulator.removeFixation(torso);
		}
	}

	public void stopClimbing() {
		RevoluteJoint joint;
		float upperTorque = 0.3f;
		float lowerTorque = 0.2f;
		joint = (RevoluteJoint) lHip;
		joint.setMotorSpeed(0);
		joint.setMaxMotorTorque(upperTorque);
		joint = (RevoluteJoint) lKnee;
		joint.setMotorSpeed(0);
		joint.setMaxMotorTorque(lowerTorque);
		joint = (RevoluteJoint) rHip;
		joint.setMotorSpeed(0);
		joint.setMaxMotorTorque(upperTorque);
		joint = (RevoluteJoint) rKnee;
		joint.setMotorSpeed(0);
		joint.setMaxMotorTorque(lowerTorque);
		joint = (RevoluteJoint) lShoulder;
		joint.setMotorSpeed(0);
		joint.setMaxMotorTorque(upperTorque);
		joint = (RevoluteJoint) lElbow;
		joint.setMotorSpeed(0);
		joint.setMaxMotorTorque(lowerTorque);
		joint = (RevoluteJoint) rShoulder;
		joint.setMotorSpeed(0);
		joint.setMaxMotorTorque(upperTorque);
		joint = (RevoluteJoint) rElbow;
		joint.setMotorSpeed(0);
		joint.setMaxMotorTorque(lowerTorque);
	}

	public void startClimbing(Body touchedBody) {
		RevoluteJoint joint;
		float torque = 2f;
		if (touchedBody == lHand || touchedBody == rHand) {
			joint = (RevoluteJoint) lHip;
			joint.setMotorSpeed(1f);
			joint.setMaxMotorTorque(torque);
			joint.enableMotor(true);
			joint = (RevoluteJoint) rHip;
			joint.setMotorSpeed(-1f);
			joint.setMaxMotorTorque(torque);
			joint.enableMotor(true);
			joint = (RevoluteJoint) lKnee;
			joint.setMotorSpeed(-1f);
			joint.setMaxMotorTorque(torque);
			joint.enableMotor(true);
			joint = (RevoluteJoint) rKnee;
			joint.setMotorSpeed(1f);
			joint.setMaxMotorTorque(torque);
			joint.enableMotor(true);
		} else if (touchedBody == lFoot || touchedBody == rFoot) {
			joint = (RevoluteJoint) lShoulder;
			joint.setMotorSpeed(1f);
			joint.setMaxMotorTorque(torque);
			joint.enableMotor(true);
			joint = (RevoluteJoint) lElbow;
			joint.setMotorSpeed(-1f);
			joint.setMaxMotorTorque(torque);
			joint.enableMotor(true);
			joint = (RevoluteJoint) rShoulder;
			joint.setMotorSpeed(-1f);
			joint.setMaxMotorTorque(torque);
			joint.enableMotor(true);
			joint = (RevoluteJoint) rElbow;
			joint.setMotorSpeed(1f);
			joint.setMaxMotorTorque(torque);
			joint.enableMotor(true);
		}
		if (torso.getUserData() != null) {
			simulator.removeFixation(torso);
		}
	}

	public void stopClimbing(Body touchedBody) {
		RevoluteJoint joint;
		float upperTorque = 0.3f;
		float lowerTorque = 0.2f;
		if (touchedBody == lHand || touchedBody == rHand) {
			joint = (RevoluteJoint) lHip;
			joint.setMotorSpeed(0);
			joint.setMaxMotorTorque(upperTorque);
			joint = (RevoluteJoint) lKnee;
			joint.setMotorSpeed(0);
			joint.setMaxMotorTorque(lowerTorque);
			joint = (RevoluteJoint) rHip;
			joint.setMotorSpeed(0);
			joint.setMaxMotorTorque(upperTorque);
			joint = (RevoluteJoint) rKnee;
			joint.setMotorSpeed(0);
			joint.setMaxMotorTorque(lowerTorque);
		} else if (touchedBody == lFoot || touchedBody == rFoot) {
			joint = (RevoluteJoint) lShoulder;
			joint.setMotorSpeed(0);
			joint.setMaxMotorTorque(upperTorque);
			joint = (RevoluteJoint) lElbow;
			joint.setMotorSpeed(0);
			joint.setMaxMotorTorque(lowerTorque);
			joint = (RevoluteJoint) rShoulder;
			joint.setMotorSpeed(0);
			joint.setMaxMotorTorque(upperTorque);
			joint = (RevoluteJoint) rElbow;
			joint.setMotorSpeed(0);
			joint.setMaxMotorTorque(lowerTorque);
		}
	}
	public Body[] getTouchableBodies() {
		return touchableBodies;
	}

	public Body getTorso() {
		return torso;
	}
}
