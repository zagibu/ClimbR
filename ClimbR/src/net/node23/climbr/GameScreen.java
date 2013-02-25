package net.node23.climbr;

import net.node23.climbr.model.World;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Screen;

public class GameScreen implements Screen {

	private World world;
	private WorldRenderer renderer;
	private WorldController controller;
	private WorldSimulator simulator;

	public GameScreen(World world) {
		super();
		this.world = world;
	}


	@Override
	public void render(float delta) {
		// controller.update(delta);
		simulator.update();
		renderer.render();
	}

	@Override
	public void resize(int width, int height) {
		renderer.setSize(width, height);
		controller.setSize(width, height);
	}

	@Override
	public void show() {
		simulator = new WorldSimulator();
		renderer = new WorldRenderer(world, simulator, true);
		renderer.setSize(480, 800);
		controller = new WorldController(world);
		controller.setSize(480, 800);
	}

	@Override
	public void hide() {
		Gdx.input.setInputProcessor(null);
	}

	@Override
	public void pause() {
		// TODO Auto-generated method stub

	}

	@Override
	public void resume() {
		// TODO Auto-generated method stub

	}

	@Override
	public void dispose() {
		// TODO Auto-generated method stub

	}

}
