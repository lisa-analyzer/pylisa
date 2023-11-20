package it.unive.ros.application;
import it.unive.lisa.program.Program;
import it.unive.pylisa.PyFrontend;
import it.unive.ros.application.exceptions.ROSNodeBuildException;


public class PythonROSNodeBuilder extends ROSNodeBuilder {


    public PythonROSNodeBuilder(String fileName) {
        super(fileName);
    }

    @Override
    protected Program getLiSAProgram() throws ROSNodeBuildException {
        try {
            PyFrontend translator = new PyFrontend(getFileName(), false);
            return translator.toLiSAProgram();
        } catch (Exception e) {
            throw new ROSNodeBuildException(e);
        }

    }
}
