package it.unive.pylisa;

public class ModuleImportDescriptor {
    private String as = null;
    private String modulePath = null;

    public ModuleImportDescriptor(String modulePath) {
        this.modulePath = modulePath;
    }

    public ModuleImportDescriptor(String modulePath, String as) {
        this(modulePath);
        this.as = as;
    }

    public String getModulePath() {
        return this.modulePath;
    }

    public String getAs() {
        return this.as;
    }

}
