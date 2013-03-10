package net.node23.climbr;

import physics.Simulator;
import net.node23.climbr.model.World;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.Screen;

public class GameScreen implements Screen {

	// TODO: doesn't really do anything with the world
	private World world;
	private Renderer renderer;
	private Controller controller;
	private Simulator simulator;

	public GameScreen(World world) {
		super();
		this.world = world;
	}


	@Override
	public void render(float delta) {
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
		// TODO: should they really be created here?
		simulator = new Simulator();
		renderer = new Renderer(world, simulator, true);
		renderer.setSize(480, 800);
		controller = new Controller(world, simulator);
		controller.setSize(480, 800);
	}

	@Override
	public void hide() {
		Gdx.input.setInputProcessor(null);
	}

	@Override
	public void pause() {
	}

	@Override
	public void resume() {
	}

	@Override
	public void dispose() {
	}

}
