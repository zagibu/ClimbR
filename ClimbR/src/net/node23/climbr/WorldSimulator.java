package net.node23.climbr;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.BodyDef.BodyType;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.Joint;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;
import com.badlogic.gdx.utils.Array;

public class WorldSimulator {

	public static int PPU = 160;
	private World simulation;
	private Array<Body> bodies = new Array<Body>();
	private Body ground;
	private Body torso;
	private RevoluteJointDef fixationDef;
	Vector2 gravity = new Vector2(0, -1.82f);
	Vector2 touchDirection = new Vector2();
	Vector2 velocity = new Vector2();
	private Body touchedBody;
	private float scale;

	public WorldSimulator() {
		float centerX = 3 / 2f;
		float centerY = 5 / 2f;

		simulation = new World(gravity, true);

		// Ground
		BodyDef groundBodyDef = new BodyDef();
		groundBodyDef.position.set(new Vector2(centerX, -0.1f));
		// groundBodyDef.angle = 0.045f;
		Body ground = simulation.createBody(groundBodyDef);
		PolygonShape groundBox = new PolygonShape();
		groundBox.setAsBox(2f, 0.1f);
		ground.createFixture(groundBox, 0.0f);
		this.ground = ground;

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
		this.torso = torso;

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

		RevoluteJointDef jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, luArm, new Vector2(centerX - 0.2f, 1.5f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0f * Math.PI);
		jointDef.lowerAngle = (float) (-0.85f * Math.PI);
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(luArm, llArm, new Vector2(centerX - 0.2f, 1.10f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0f * Math.PI);
		jointDef.lowerAngle = (float) (-0.85f * Math.PI);
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(llArm, lHand, new Vector2(centerX - 0.2f, 0.8f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.05f * Math.PI);
		jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, ruArm, new Vector2(centerX + 0.2f, 1.5f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.85f * Math.PI);
		jointDef.lowerAngle = (float) (0f * Math.PI);
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(ruArm, rlArm, new Vector2(centerX + 0.2f, 1.10f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.85f * Math.PI);
		jointDef.lowerAngle = (float) (0f * Math.PI);
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(rlArm, rHand, new Vector2(centerX + 0.2f, 0.8f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.05f * Math.PI);
		jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, luLeg, new Vector2(centerX - 0.1f, 1f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.1f * Math.PI);
		jointDef.lowerAngle = (float) (-0.75f * Math.PI);
		// jointDef.enableMotor = true;
		// jointDef.motorSpeed = 0f;
		// jointDef.maxMotorTorque = 1f;
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(luLeg, llLeg, new Vector2(centerX - 0.1f, 0.5f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (0f * Math.PI);
		jointDef.upperAngle = (float) (0.85f * Math.PI);
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(llLeg, lFoot, new Vector2(centerX - 0.1f, 0.1f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		jointDef.upperAngle = (float) (0.05f * Math.PI);
		// jointDef.enableMotor = true;
		// jointDef.motorSpeed = 0f;
		// jointDef.maxMotorTorque = 1f;
		simulation.createJoint(jointDef);

		createFixation(lFoot);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(torso, ruLeg, new Vector2(centerX + 0.1f, 1f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.6f * Math.PI);
		jointDef.lowerAngle = (float) (-0.25f * Math.PI);
		// jointDef.enableMotor = true;
		// jointDef.motorSpeed = 0f;
		// jointDef.maxMotorTorque = 1f;
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(ruLeg, rlLeg, new Vector2(centerX + 0.1f, 0.5f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (-0.85f * Math.PI);
		jointDef.upperAngle = (float) (0f * Math.PI);
		simulation.createJoint(jointDef);

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

		jointDef = new RevoluteJointDef();
		jointDef.initialize(rlLeg, rFoot, new Vector2(centerX + 0.1f, 0.1f));
		jointDef.collideConnected = false;
		jointDef.enableLimit = true;
		jointDef.lowerAngle = (float) (-0.05f * Math.PI);
		jointDef.upperAngle = (float) (0.05f * Math.PI);
		// jointDef.enableMotor = true;
		// jointDef.motorSpeed = 0f;
		// jointDef.maxMotorTorque = 1f;
		simulation.createJoint(jointDef);

		createFixation(rFoot);

		bodies.add(lHand);
		bodies.add(rHand);
		bodies.add(lFoot);
		bodies.add(rFoot);
	}

	public void update() {
		if (touchedBody != null) {
			scale = gravity.y / 2f;
			if (scale < 0) {
				scale *= -1f;
			}
			if (scale < 3) {
				scale = 3f;
			}
			velocity = touchDirection.cpy().sub(touchedBody.getPosition());
			touchedBody.setLinearVelocity(velocity.nor().mul(scale));
		}
		simulation.step(1 / 60f, 6, 2);
	}

	public void touchBody(Body body, int screenX, int screenY) {
		touchedBody = body;
		removeFixation(body);
		updateTouch(screenX, screenY);
	}

	public void untouchBody() {
		if (touchedBody != null) {
			touchedBody.setLinearVelocity(0, 0);
			createFixation(touchedBody);
			touchedBody = null;
		}
	}

	public void updateTouch(int screenX, int screenY) {
		if (touchedBody != null) {
			touchDirection.set(screenX / (float) PPU, screenY / (float) PPU);
		}
	}

	public void removeFixation() {
		if (touchedBody != null) {
			touchedBody.setLinearVelocity(0, 0);
			removeFixation(touchedBody);
			touchedBody = null;
		}
		
	}
	public void removeFixation(Body body) {
		if (body.getUserData() != null) {
			simulation.destroyJoint((Joint) body.getUserData());
			body.setUserData(null);
		}
	}

	public void createFixation(Body body) {
		fixationDef = new RevoluteJointDef();
		fixationDef.initialize(body, ground, body.getPosition());
		fixationDef.enableMotor = true;
		fixationDef.motorSpeed = 0;
		fixationDef.maxMotorTorque = 1f;
		body.setUserData(simulation.createJoint(fixationDef));
	}

	public void decreaseGravity() {
		gravity.set(0, gravity.y + 1);
		if (gravity.y > 0) {
			gravity.set(0, 0);
		}
		simulation.setGravity(gravity);
	}

	public void increaseGravity() {
		gravity.set(0, gravity.y - 1);
		if (gravity.y < -9.82) {
			gravity.set(0, -9.82f);
		}
		simulation.setGravity(gravity);
	}

	public void toggleGravity() {
		if (gravity.y != 0) {
			gravity.set(0, 0);
		} else {
			gravity.set(0, -9.82f);
		}
		simulation.setGravity(gravity);
	}

	public void resetGravity() {
		if (gravity == null) {
			gravity = new Vector2();
		}
		gravity.set(0, -9.82f);
		simulation.setGravity(gravity);
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
