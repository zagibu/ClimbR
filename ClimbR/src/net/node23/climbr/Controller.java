package net.node23.climbr;

import net.node23.climbr.model.Hold;
import net.node23.climbr.model.World;
import physics.Simulator;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Pixmap;
import com.badlogic.gdx.physics.box2d.Body;

public class Controller implements InputProcessor {

	private World world;
	private Simulator simulator;
	private Pixmap handholds;
	private Pixmap footholds;
	private int height;

	public Controller(World world, Simulator simulator) {
		this.world = world;
		this.simulator = simulator;
		handholds = new Pixmap(Gdx.files.internal(world.getHandholdsMap()));
		footholds = new Pixmap(Gdx.files.internal(world.getFootholdsMap()));
		Gdx.input.setInputProcessor(this);
	}

	public float getHandholdQuality(int x, int y) {
		return getHoldQuality(x, y, Hold.Type.HANDS);
	}

	public float getFootholdQuality(int x, int y) {
		return getHoldQuality(x, y, Hold.Type.FEET);
	}

	public float getHoldQuality(int x, int y, Hold.Type type) {
		y = height - y;
		for (Hold hold : world.getHolds()) {
			if (hold.getX() <= x && x <= hold.getX() + Hold.SIZE
					&& hold.getY() <= y && y <= hold.getY() + Hold.SIZE) {
				return getHoldQuality(hold, x, y, type);
			}
		}
		return 0;
	}

	private float getHoldQuality(Hold hold, int x, int y, Hold.Type type) {
		// TODO: assumes hands and feet heightmaps are same size
		int pX = hold.getIndex() * Hold.SIZE % handholds.getWidth() + x - hold.getX();
		int pY = hold.getIndex() * Hold.SIZE / handholds.getWidth() * Hold.SIZE
				+ Hold.SIZE - (y - hold.getY());
		int rgbaa = 0;
		if (type == Hold.Type.HANDS) {
			rgbaa = handholds.getPixel(pX, pY);
		} else if (type == Hold.Type.FEET) {
			rgbaa = footholds.getPixel(pX, pY);
		}
		return (((rgbaa & 0xff000000) >>> 24) + ((rgbaa & 0x00ff0000) >>> 16) + ((rgbaa & 0x0000ff00) >>> 8)) / (3 * 255f);
	}

	public void setSize(int width, int height) {
		this.height = height;
	}

	@Override
	public boolean keyDown(int keycode) {
		return false;
	}

	@Override
	public boolean keyUp(int keycode) {
		return false;
	}

	@Override
	public boolean keyTyped(char character) {
		if (character == '+') {
			simulator.increaseGravity();
		}
		if (character == '-') {
			simulator.decreaseGravity();
		}
		if (character == 'g') {
			simulator.toggleGravity();
		}
		if (character == 'x') {
			simulator.removeAllFixations();
		}
		return false;
	}

	@Override
	public boolean touchDown(int screenX, int screenY, int pointer, int button) {
		screenY = height - screenY;
		float smallestDistance = 0.1f;
		Body closestBody = null;
		for (Body body : simulator.getPlayer().getTouchableBodies()) {
			float dX = Math.abs(screenX / (float) Simulator.PPU
					- body.getPosition().x);
			float dY = Math.abs(screenY / (float) Simulator.PPU
					- body.getPosition().y);
			if (dX * dX + dY * dY < smallestDistance) {
				smallestDistance = dX * dX + dY * dY;
				closestBody = body;
			}
		}
		if (closestBody != null) {
			simulator.touchBody(closestBody, screenX, screenY, button);
		}
		return false;
	}

	@Override
	public boolean touchDragged(int screenX, int screenY, int pointer) {
		screenY = height - screenY;

		simulator.updateTouch(screenX, screenY);
		return false;
	}

	@Override
	public boolean touchUp(int screenX, int screenY, int pointer, int button) {
		if (button == 0) {
			simulator.untouchBody();
		} else if (button == 1) {
			simulator.removeFixation();
		}
		return false;
	}

	@Override
	public boolean mouseMoved(int screenX, int screenY) {
		return false;
	}

	@Override
	public boolean scrolled(int amount) {
		return false;
	}
}
