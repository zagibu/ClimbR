package net.node23.climbr;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.BodyDef.BodyType;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;
import com.badlogic.gdx.utils.Array;

public class WorldSimulator {

	public static int PPU = 160;
	private World simulation = new World(new Vector2(0f, -9.82f), true);
	private Array<Body> bodies = new Array<Body>();
	private Body ground;
	private Body torso;

	public WorldSimulator() {
		float centerX = 3 / 2f;
		float centerY = 5 / 2f;

		// Ground
		BodyDef groundBodyDef = new BodyDef();
		groundBodyDef.position.set(new Vector2(centerX, -0.1f));
		groundBodyDef.angle = 0.045f;
		Body ground = simulation.createBody(groundBodyDef);
		PolygonShape groundBox = new PolygonShape();
		groundBox.setAsBox(2f, 0.1f);
		ground.createFixture(groundBox, 0.0f);

		// Torso
		BodyDef bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX, 1.25f);
		Body torso = simulation.createBody(bodyDef);
		PolygonShape shape = new PolygonShape();
		shape.setAsBox(0.2f, 0.25f);
		FixtureDef fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		torso.createFixture(fixtureDef);

		// Left upper arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX - 0.2f, 1.3f);
		Body luArm = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.075f, 0.2f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		luArm.createFixture(fixtureDef);

		RevoluteJointDef jointDev = new RevoluteJointDef();
		jointDev.initialize(torso, luArm, new Vector2(centerX - 0.2f, 1.5f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.upperAngle = (float) (0f * Math.PI);
		jointDev.lowerAngle = (float) (-0.85f * Math.PI);
		simulation.createJoint(jointDev);

		// Left lower arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX - 0.2f, 0.95f);
		Body llArm = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f, 0.15f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		llArm.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(luArm, llArm, new Vector2(centerX - 0.2f, 1.10f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.upperAngle = (float) (0f * Math.PI);
		jointDev.lowerAngle = (float) (-0.85f * Math.PI);
		simulation.createJoint(jointDev);

		// Left hand
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX - 0.2f, 0.75f);
		Body lHand = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f, 0.05f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		lHand.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(llArm, lHand, new Vector2(centerX - 0.2f, 0.8f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.upperAngle = (float) (0.05f * Math.PI);
		jointDev.lowerAngle = (float) (-0.05f * Math.PI);
		simulation.createJoint(jointDev);

		// Right upper arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX + 0.2f, 1.3f);
		Body ruArm = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.075f, 0.2f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		ruArm.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(torso, ruArm, new Vector2(centerX + 0.2f, 1.5f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.upperAngle = (float) (0.85f * Math.PI);
		jointDev.lowerAngle = (float) (0f * Math.PI);
		simulation.createJoint(jointDev);

		// Right lower arm
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX + 0.2f, 0.95f);
		Body rlArm = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f, 0.15f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		rlArm.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(ruArm, rlArm, new Vector2(centerX + 0.2f, 1.10f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.upperAngle = (float) (0f * Math.PI);
		jointDev.lowerAngle = (float) (-0.85f * Math.PI);
		simulation.createJoint(jointDev);

		// Right hand
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX + 0.2f, 0.75f);
		Body rHand = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.05f, 0.05f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		rHand.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(rlArm, rHand, new Vector2(centerX + 0.2f, 0.8f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.upperAngle = (float) (0.05f * Math.PI);
		jointDev.lowerAngle = (float) (-0.05f * Math.PI);
		simulation.createJoint(jointDev);

		// Left upper leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX - 0.1f, 0.75f);
		Body luLeg = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.085f, 0.25f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		luLeg.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(torso, luLeg, new Vector2(centerX - 0.1f, 1f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.upperAngle = (float) (0.1f * Math.PI);
		jointDev.lowerAngle = (float) (-0.75f * Math.PI);
		jointDev.enableMotor = true;
		jointDev.motorSpeed = 0f;
		jointDev.maxMotorTorque = 1f;
		simulation.createJoint(jointDev);

		// Left lower leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX - 0.1f, 0.275f);
		Body llLeg = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f, 0.225f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		llLeg.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(luLeg, llLeg, new Vector2(centerX - 0.1f, 0.5f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.lowerAngle = (float) (0f * Math.PI);
		jointDev.upperAngle = (float) (0.85f * Math.PI);
		simulation.createJoint(jointDev);

		// Left foot
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX - 0.1f, 0.05f);
		Body lFoot = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f, 0.05f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		lFoot.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(llLeg, lFoot, new Vector2(centerX - 0.1f, 0.1f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.lowerAngle = (float) (-0.05f * Math.PI);
		jointDev.upperAngle = (float) (0.05f * Math.PI);
		jointDev.enableMotor = true;
		jointDev.motorSpeed = 0f;
		jointDev.maxMotorTorque = 1f;
		simulation.createJoint(jointDev);

		// Right upper leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX + 0.1f, 0.75f);
		Body ruLeg = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.085f, 0.25f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		ruLeg.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(torso, ruLeg, new Vector2(centerX + 0.1f, 1f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.upperAngle = (float) (0.25f * Math.PI);
		jointDev.lowerAngle = (float) (-0.6f * Math.PI);
		jointDev.enableMotor = true;
		jointDev.motorSpeed = 0f;
		jointDev.maxMotorTorque = 1f;
		simulation.createJoint(jointDev);

		// Right lower leg
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX + 0.1f, 0.275f);
		Body rlLeg = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f, 0.225f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		rlLeg.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(ruLeg, rlLeg, new Vector2(centerX + 0.1f, 0.5f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.lowerAngle = (float) (-0.85f * Math.PI);
		jointDev.upperAngle = (float) (0f * Math.PI);
		simulation.createJoint(jointDev);

		// Right foot
		bodyDef = new BodyDef();
		bodyDef.type = BodyType.DynamicBody;
		bodyDef.position.set(centerX + 0.1f, 0.05f);
		Body rFoot = simulation.createBody(bodyDef);
		shape = new PolygonShape();
		shape.setAsBox(0.07f, 0.05f);
		fixtureDef = new FixtureDef();
		fixtureDef.shape = shape;
		fixtureDef.density = 1.0f;
		fixtureDef.friction = 0.5f;
		fixtureDef.restitution = 0.2f;
		rFoot.createFixture(fixtureDef);

		jointDev = new RevoluteJointDef();
		jointDev.initialize(rlLeg, rFoot, new Vector2(centerX + 0.1f, 0.1f));
		jointDev.collideConnected = false;
		jointDev.enableLimit = true;
		jointDev.lowerAngle = (float) (-0.05f * Math.PI);
		jointDev.upperAngle = (float) (0.05f * Math.PI);
		jointDev.enableMotor = true;
		jointDev.motorSpeed = 0f;
		jointDev.maxMotorTorque = 1f;
		simulation.createJoint(jointDev);

		this.ground = ground;
		this.torso = torso;
		bodies.add(lHand);
		bodies.add(rHand);
		bodies.add(lFoot);
		bodies.add(rFoot);
	}

	public void update() {
		simulation.step(1 / 60f, 6, 2);
	}

	public com.badlogic.gdx.physics.box2d.World getSimulation() {
		return simulation;
	}

	public Array<Body> getBodies() {
		return bodies;
	}
	
	public Body getGround() {
		return ground;
	}

	public Body getTorso() {
		return torso;
	}
}
