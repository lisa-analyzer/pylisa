package it.unive.pylisa.cfg;

import it.unive.lisa.program.cfg.CodeLocation;

public class StarParameter extends PyParameter {
	public StarParameter(
			CodeLocation location) {
		super(location, "*");
	}
}
