package net.node23.climbr.model;

import com.badlogic.gdx.utils.Array;

public class World {

	private String backgroundTexture;
	private String holdsTexture;
	private String holdsHandsMap;
	private String holdsFeetMap;
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

	public String getHoldsHandsMap() {
		return holdsHandsMap;
	}

	public String getHoldsFeetMap() {
		return holdsFeetMap;
	}
}
