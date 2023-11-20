package it.unive.ros.application;

import it.unive.lisa.program.Program;
import it.unive.ros.application.exceptions.ROSNodeBuildException;


public abstract class ROSNodeBuilder {

    private String fileName;

    public ROSNodeBuilder(String fileName) {
        this.fileName = fileName;
    }

    abstract Program getLiSAProgram() throws ROSNodeBuildException;

    public String getFileName() {
        return this.fileName;
    }
}
