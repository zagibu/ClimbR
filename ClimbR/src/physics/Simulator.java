package physics;

import com.badlogic.gdx.math.Vector2;
import com.badlogic.gdx.physics.box2d.Body;
import com.badlogic.gdx.physics.box2d.BodyDef;
import com.badlogic.gdx.physics.box2d.FixtureDef;
import com.badlogic.gdx.physics.box2d.Joint;
import com.badlogic.gdx.physics.box2d.PolygonShape;
import com.badlogic.gdx.physics.box2d.World;
import com.badlogic.gdx.physics.box2d.joints.RevoluteJointDef;

public class Simulator {

	public static float WIDTH = 3;
	public static float HEIGHT = 5;
	public static int PPU = 160;
	private World simulation;
	private Body ground;
	private RevoluteJointDef jointDef;
	Vector2 gravity = new Vector2(0, -9.82f);
	Vector2 touchDirection = new Vector2();
	Vector2 velocity = new Vector2();
	private Body touchedBody;
	private float scale;
	private PhysicalPlayer player;
	private int numberOfHolds = 0;

	public Simulator() {
		simulation = new World(gravity, true);

		// Ground
		BodyDef groundBodyDef = new BodyDef();
		groundBodyDef.position.set(new Vector2(WIDTH / 2f, -0.1f));
		// groundBodyDef.angle = 0.045f;
		Body ground = simulation.createBody(groundBodyDef);
		PolygonShape groundBox = new PolygonShape();
		groundBox.setAsBox(2f, 0.1f);
		FixtureDef fixtureDef = new FixtureDef();
		fixtureDef.shape = groundBox;
		fixtureDef.density = 0;
		fixtureDef.filter.categoryBits = 0x1;
		ground.createFixture(fixtureDef);
		this.ground = ground;

		player = new PhysicalPlayer(this);
	}

	public void update() {
		if (touchedBody != null) {
			// TODO: hard to understand
			scale = gravity.y / 2f;
			if (scale < 0) {
				scale *= -1f;
			}
			if (scale < 3) {
				scale = 3f;
			}
			if (touchedBody == player.getTorso()) {
				if (getNumberOfHolds() < 2) {
					scale = 0.1f;
				}
			}
			touchedBody.setLinearVelocity(0, 0);
			velocity = touchDirection.cpy().sub(touchedBody.getPosition());
			if (velocity.len() > 0.04f) {
				touchedBody.setLinearVelocity(velocity.nor().mul(scale));
			}
		}
		checkFeetAngle();
		simulation.step(1 / 60f, 6, 2);
	}

	public void checkFeetAngle() {
		if (player.getLeftFoot().getUserData() != null) {
			if (player.getLeftFoot().getAngle() < -1 * Math.PI / 2f || player.getLeftFoot().getAngle() > Math.PI / 2f) {
				removeFixation(player.getLeftFoot());
				player.loosenLimb(player.getLeftFoot());
			}
		}
		if (player.getRightFoot().getUserData() != null) {
			if (player.getRightFoot().getAngle() < -1 * Math.PI / 2f || player.getRightFoot().getAngle() > Math.PI / 2f) {
				removeFixation(player.getRightFoot());
				player.loosenLimb(player.getRightFoot());
			}
		}
	}

	public void touchBody(Body body, int screenX, int screenY, int button) {
		touchedBody = body;
		removeFixation(touchedBody);
		player.loosenLimb(body);
		if (button == 0) {
			disableGravity(body);
		}
		updateTouch(screenX, screenY);
	}

	public void untouchBody() {
		if (touchedBody != null) {
			touchedBody.setLinearVelocity(0, 0);
			if (touchedBody != player.getTorso()) {
				createFixation(touchedBody);
				player.tightenLimb(touchedBody);
			}
			enableGravity(touchedBody);
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
			// player.stopClimbing(touchedBody);
			touchedBody.setLinearVelocity(0, 0);
			removeFixation(touchedBody);
			touchedBody = null;
		}

	}

	public void removeFixation(Body body) {
		if (body.getUserData() != null) {
			simulation.destroyJoint((Joint) body.getUserData());
			body.setUserData(null);
			numberOfHolds--;
			System.out.println(numberOfHolds);
		}
	}

	public void removeAllFixations() {
		for (Body body : player.getTouchableBodies()) {
			removeFixation(body);
		}
	}

	public void createFixation(Body body) {
		jointDef = new RevoluteJointDef();
		jointDef.initialize(body, ground, body.getPosition());
		jointDef.enableLimit = true;
		jointDef.upperAngle = (float) (0.5f * Math.PI);
		jointDef.lowerAngle = (float) (-0.5f * Math.PI);
		jointDef.enableMotor = true;
		jointDef.motorSpeed = 0;
		jointDef.maxMotorTorque = 2f;
		body.setUserData(simulation.createJoint(jointDef));
		numberOfHolds++;
	}

	public void enableGravity(Body body) {
		System.out.println("enabling gravity for " + body);
		changeGravity(body, 1);
	}

	public void disableGravity(Body body) {
		System.out.println("disabling gravity for " + body);
		changeGravity(body, 0);
	}

	private void changeGravity(Body body, float gravity) {
		if (body == player.getLeftHand()) {
			player.getLeftUpperArm().setGravityScale(gravity);
			player.getLeftLowerArm().setGravityScale(gravity);
			player.getLeftHand().setGravityScale(gravity);
		}
		if (body == player.getRightHand()) {
			player.getRightUpperArm().setGravityScale(gravity);
			player.getRightLowerArm().setGravityScale(gravity);
			player.getRightHand().setGravityScale(gravity);
		}
		if (body == player.getLeftFoot()) {
			player.getLeftUpperLeg().setGravityScale(gravity);
			player.getLeftLowerLeg().setGravityScale(gravity);
			player.getLeftFoot().setGravityScale(gravity);
		}
		if (body == player.getRightFoot()) {
			player.getRightUpperLeg().setGravityScale(gravity);
			player.getRightLowerLeg().setGravityScale(gravity);
			player.getRightFoot().setGravityScale(gravity);
		}
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

	public Body getGround() {
		return ground;
	}

	public PhysicalPlayer getPlayer() {
		return player;
	}

	private int getNumberOfHolds() {
		return numberOfHolds;
	}
}
