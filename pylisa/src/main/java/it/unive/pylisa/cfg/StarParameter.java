package it.unive.pylisa.cfg;

import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.Parameter;

public class StarParameter extends Parameter {
	public StarParameter(
			CodeLocation location) {
		super(location, "*");
	}
}
