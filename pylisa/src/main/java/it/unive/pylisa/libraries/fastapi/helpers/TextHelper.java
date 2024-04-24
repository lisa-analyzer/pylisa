package it.unive.pylisa.libraries.fastapi.helpers;

import it.unive.lisa.program.Unit;
import lombok.experimental.UtilityClass;

@UtilityClass
public class TextHelper {

    public String getFilenameFromUnit(Unit unit) {

        String filepath = unit.getName();

        if (filepath == null || filepath.isEmpty()) {
            return "";
        }

        int lastSlashIndex = filepath.lastIndexOf('/');

        if (lastSlashIndex == -1) {
            return filepath;
        }

        return filepath.substring(lastSlashIndex + 1);
    }
}
