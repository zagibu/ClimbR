package net.node23.climbr.model;

import com.badlogic.gdx.utils.Array;

public class World {

	private String backgroundTexture;
	private String holdsTexture;
	private String handholdsMap;
	private String footholdsMap;
	private Array<Hold> holds;

	public Array<Hold> getHolds() {
		return holds;
	}

	public String getBackgroundTexture() {
		return backgroundTexture;
	}

	public String getHoldsTexture() {
		return holdsTexture;
	}

	public String getHandholdsMap() {
		return handholdsMap;
	}

	public String getFootholdsMap() {
		return footholdsMap;
	}
}
