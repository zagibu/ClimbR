package net.node23.climbr.util;

import net.node23.climbr.model.Hold;
import net.node23.climbr.model.World;

import com.badlogic.gdx.Gdx;
import com.badlogic.gdx.utils.Json;

public class Loader {

	public static World loadWorld(String string) {
		Json json = new Json();
		json.setElementType(World.class, "holds", Hold.class);
		return json.fromJson(World.class, Gdx.files.internal(string + ".json").readString());
	}

}
