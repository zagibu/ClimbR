package net.node23.climbr;

import net.node23.climbr.util.Loader;

import com.badlogic.gdx.Game;

public class ClimbR extends Game {

	@Override
	public void create() {
		setScreen(new GameScreen(Loader.loadWorld("indoor1")));
	}
	
}
