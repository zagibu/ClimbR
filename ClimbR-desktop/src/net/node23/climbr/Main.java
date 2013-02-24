package net.node23.climbr;

import com.badlogic.gdx.backends.lwjgl.LwjglApplication;
import com.badlogic.gdx.backends.lwjgl.LwjglApplicationConfiguration;

public class Main {
	public static void main(String[] args) {
		LwjglApplicationConfiguration cfg = new LwjglApplicationConfiguration();
		cfg.title = "ClimbR";
		cfg.useGL20 = false;
		cfg.width = 480;
		cfg.height = 800;
		
		new LwjglApplication(new ClimbR(), cfg);
	}
}
