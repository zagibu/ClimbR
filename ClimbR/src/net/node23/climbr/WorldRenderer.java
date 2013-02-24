package net.node23.climbr;

import net.node23.climbr.model.Hold;
import net.node23.climbr.model.World;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL10;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.Texture.TextureWrap;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.utils.Array;

public class WorldRenderer {

	private World world;
	private boolean debug;
	private SpriteBatch spriteBatch;
	Sprite background;
	Array<Sprite> holds = new Array<Sprite>();

	public WorldRenderer(World world, boolean debug) {
		//Gdx.graphics.setContinuousRendering(false);
		this.world = world;
		this.debug = debug;
		spriteBatch = new SpriteBatch();
		loadTextures();
	}

	private void loadTextures() {
		Texture backgroundTexture = new Texture(Gdx.files.internal(world.getBackgroundTexture()));
		backgroundTexture.setWrap(TextureWrap.Repeat, TextureWrap.Repeat);
		background = new Sprite(backgroundTexture, 0, 0, 512, 800);
		Texture holdTexture = new Texture(Gdx.files.internal(world.getHoldsTexture()));
		for (Hold hold : world.getHolds()) {
			int x = hold.getIndex() * 64 % 512;
			int y = hold.getIndex() * 64 / 512 * 64;
			Sprite sprite = new Sprite(holdTexture, x, y, 64, 64);
			sprite.setX(hold.getX());
			sprite.setY(hold.getY());
			holds.add(sprite);
		}
	}

	public void render() {
		Gdx.gl.glClearColor(1f, 1f, 1f, 1);
		Gdx.gl.glClear(GL10.GL_COLOR_BUFFER_BIT);

		spriteBatch.begin();
		background.draw(spriteBatch);
		for (Sprite sprite : holds) {
			sprite.draw(spriteBatch);
		}
		spriteBatch.end();
		if (debug)
			drawDebug();
	}

	private void drawDebug() {
		// TODO Auto-generated method stub

	}

	public void setSize(int width, int height) {
		//cam.setToOrtho(false, width, height);
		//spriteBatch.setProjectionMatrix(cam.combined);
	}

}
