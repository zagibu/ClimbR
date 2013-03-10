package net.node23.climbr;

import physics.Simulator;
import net.node23.climbr.model.Hold;
import net.node23.climbr.model.World;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.graphics.GL10;
import com.badlogic.gdx.graphics.Texture;
import com.badlogic.gdx.graphics.Texture.TextureWrap;
import com.badlogic.gdx.graphics.g2d.Sprite;
import com.badlogic.gdx.graphics.g2d.SpriteBatch;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer;
import com.badlogic.gdx.graphics.glutils.ShapeRenderer.ShapeType;
import com.badlogic.gdx.math.Matrix4;
import com.badlogic.gdx.physics.box2d.Box2DDebugRenderer;
import com.badlogic.gdx.utils.Array;

public class Renderer {

	private World world;
	Simulator simulator;
	private boolean debug;
	ShapeRenderer debugRenderer;
	Box2DDebugRenderer box2dRenderer;
	private SpriteBatch spriteBatch;
	Sprite background;
	Array<Sprite> holds = new Array<Sprite>();
	Matrix4 projectionMatrix;

	public Renderer(World world, Simulator simulator, boolean debug) {
		// Gdx.graphics.setContinuousRendering(false);
		this.world = world;
		this.simulator = simulator;
		this.debug = debug;
		debugRenderer = new ShapeRenderer();
		box2dRenderer = new Box2DDebugRenderer();
		spriteBatch = new SpriteBatch();
		projectionMatrix = spriteBatch.getProjectionMatrix().cpy();
		projectionMatrix.scale(Simulator.PPU, Simulator.PPU,
				Simulator.PPU);
		loadTextures();
	}

	private void loadTextures() {
		Texture backgroundTexture = new Texture(Gdx.files.internal(world
				.getBackgroundTexture()));
		backgroundTexture.setWrap(TextureWrap.Repeat, TextureWrap.Repeat);
		background = new Sprite(backgroundTexture, 0, 0, 512, 800);
		Texture holdTexture = new Texture(Gdx.files.internal(world
				.getHoldsTexture()));
		for (Hold hold : world.getHolds()) {
			int x = hold.getIndex() * Hold.SIZE % holdTexture.getWidth();
			int y = hold.getIndex() * Hold.SIZE / holdTexture.getWidth()
					* Hold.SIZE;
			Sprite sprite = new Sprite(holdTexture, x, y, Hold.SIZE, Hold.SIZE);
			sprite.setX(hold.getX());
			sprite.setY(hold.getY());
			holds.add(sprite);
		}
	}

	public void render() {
		Gdx.gl.glClearColor(0f, 0f, 0f, 1);
		Gdx.gl.glClear(GL10.GL_COLOR_BUFFER_BIT);

		spriteBatch.begin();
		if (!debug) {
			background.draw(spriteBatch);
			for (Sprite sprite : holds) {
				sprite.draw(spriteBatch);
			}
		}
		spriteBatch.end();
		if (debug)
			drawDebug();
	}

	private void drawDebug() {
		debugRenderer.begin(ShapeType.Rectangle);
		debugRenderer.setColor(1f, 0f, 1f, 1f);
		debugRenderer.rect(background.getX(), background.getY(),
				background.getWidth(), background.getHeight());
		for (Sprite hold : holds) {
			debugRenderer.rect(hold.getX(), hold.getY(), hold.getWidth(),
					hold.getHeight());
		}
		debugRenderer.end();
		box2dRenderer.render(simulator.getSimulation(), projectionMatrix);
	}

	public void setSize(int width, int height) {
		// cam.setToOrtho(false, width, height);
		// spriteBatch.setProjectionMatrix(cam.combined);
	}

}
