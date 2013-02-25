package net.node23.climbr;

import net.node23.climbr.model.Hold;
import net.node23.climbr.model.World;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.InputProcessor;
import com.badlogic.gdx.graphics.Pixmap;

public class WorldController implements InputProcessor {

	private World world;
	private Pixmap handholds;
	private Pixmap footholds;
	private int height;
	private int width;
	
	public WorldController(World world) {
		this.world = world;
		handholds = new Pixmap(Gdx.files.internal(world.getHandholdsMap()));
		footholds = new Pixmap(Gdx.files.internal(world.getFootholdsMap()));
		Gdx.input.setInputProcessor(this);
	}

	public int getHandholdQuality(int x, int y) {
		return getHoldQuality(x, y, Hold.Type.HANDS);
	}

	public int getFootholdQuality(int x, int y) {
		return getHoldQuality(x, y, Hold.Type.FEET);
	}

	public int getHoldQuality(int x, int y, Hold.Type type) {
		y = height - y;
		for (Hold hold : world.getHolds()) {
			if (hold.getX() <= x && x <= hold.getX() + Hold.SIZE && hold.getY() <= y && y <= hold.getY() + Hold.SIZE) {
				return getHoldQuality(hold, x, y, type);
			}
		}
		return 0;
	}

	private int getHoldQuality(Hold hold, int x, int y, Hold.Type type) {
		int dX = x - hold.getX();
		int dY = y - hold.getY();
		// FIXME: Assuming handholds and footholds pixmaps are same size
		int pX = hold.getIndex() * Hold.SIZE % handholds.getWidth() + dX;
		int pY = hold.getIndex() * Hold.SIZE / handholds.getWidth() * Hold.SIZE + Hold.SIZE - dY;

		if (type == Hold.Type.HANDS) {
			return handholds.getPixel(pX, pY);
		} else if (type == Hold.Type.FEET) {
			return footholds.getPixel(pX, pY);
		}
		return 0;
	}

	public void setSize(int width, int height) {
		this.width = width;
		this.height = height;
	}

	@Override
	public boolean keyDown(int keycode) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean keyUp(int keycode) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean keyTyped(char character) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean touchDown(int screenX, int screenY, int pointer, int button) {
		System.out.println("touchDown");
		return false;
	}

	@Override
	public boolean touchUp(int screenX, int screenY, int pointer, int button) {
		System.out.println("touchUp");
		return false;
	}

	@Override
	public boolean touchDragged(int screenX, int screenY, int pointer) {
		System.out.println("touchDragged");
		return false;
	}

	@Override
	public boolean mouseMoved(int screenX, int screenY) {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public boolean scrolled(int amount) {
		// TODO Auto-generated method stub
		return false;
	}
}
