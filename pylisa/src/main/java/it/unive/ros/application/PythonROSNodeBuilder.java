package it.unive.ros.application;

import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.ros.application.exceptions.ROSNodeBuildException;
import java.util.List;

public class PythonROSNodeBuilder extends ROSNodeBuilder {

	public PythonROSNodeBuilder(
			String fileName) {
		super(fileName);
	}

	@Override
	protected Program getLiSAProgram() throws ROSNodeBuildException {
		try {
			PyFrontend translator = new PyFrontend();
			return translator.parseFromListOfFile(List.of(getFileName()));
		} catch (Exception e) {
			throw new ROSNodeBuildException(e);
		}

	}
}
