package it.unive.pylisa;

import java.util.Collection;
import java.util.HashSet;

import it.unive.lisa.checks.warnings.Warning;
import it.unive.lisa.checks.warnings.WarningWithLocation;
import it.unive.lisa.program.SourceCodeLocation;
import it.unive.lisa.program.cfg.CodeLocation;

public class Stub {
    public static Collection<Warning> analyze(String filePath) {
        Collection<Warning> result = new HashSet<>();
        result.add(new PyWarningWithLocation(
                new SourceCodeLocation(filePath, 1, 10),
                "bad warning at line 1!"));
        result.add(new PyWarningWithLocation(
                new SourceCodeLocation(filePath, 2, 4),
                "bad warning at line 2!"));
        result.add(new PyWarningWithLocation(
                new SourceCodeLocation(filePath, 3, 12),
                "bad warning at line 3!"));
        result.add(new PyWarningWithLocation(
                new SourceCodeLocation(filePath, 4, 18),
                "bad warning at line 4!"));
        result.add(new PyWarningWithLocation(
                new SourceCodeLocation(filePath, 5, 3),
                "bad warning at line 5!"));
        result.add(new PyWarningWithLocation(
                new SourceCodeLocation(filePath, 6, 9),
                "bad warning at line 6!"));
        result.add(new PyWarningWithLocation(
                new SourceCodeLocation(filePath, 7, 18),
                "bad warning at line 7!"));
        return result;
    }

    private static class PyWarningWithLocation extends  WarningWithLocation {

        protected PyWarningWithLocation(CodeLocation location, String message) {
            super(location, message);
        }

        @Override
        public String toString() {
            return super.getLocation()+": "+super.getMessage();
        }
    }
}
