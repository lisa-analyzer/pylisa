package it.unive.pylisa.analysis;

import java.util.List;

public class ExplicitImport extends Import {
    private List<String> importOnly;
    public ExplicitImport(String unitName, List<String> importOnly) {
        super(unitName);
        this.importOnly = importOnly;
    }
}
