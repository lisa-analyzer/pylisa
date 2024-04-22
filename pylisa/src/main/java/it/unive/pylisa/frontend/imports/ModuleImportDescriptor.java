package it.unive.pylisa.frontend.imports;

public class ModuleImportDescriptor {
    private String as = null;
    private String modulePath = null;

    private boolean namespace = false;

    private ModuleImportDescriptor(String modulePath) {
        this.modulePath = modulePath;
    }

    public ModuleImportDescriptor(String modulePath, String as) {
        this(modulePath);
        this.as = as;
    }

    public ModuleImportDescriptor(String modulePath, String as, boolean namespace) {
        this(modulePath, as);
        this.namespace = namespace;
    }

    public String getModulePath() {
        return this.modulePath;
    }

    public String getAs() {
        return this.as;
    }

    public boolean isNamespace() {
        return namespace;
    }
    public void setAs(String as) {
        this.as = as;
    }

}
