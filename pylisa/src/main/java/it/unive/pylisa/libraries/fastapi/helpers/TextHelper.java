package it.unive.pylisa.libraries.fastapi.helpers;

import it.unive.lisa.program.Unit;
import java.io.IOException;
import java.io.InputStream;
import java.nio.charset.StandardCharsets;
import java.util.Scanner;
import lombok.experimental.UtilityClass;

@UtilityClass
public class TextHelper {

	public String getFilenameFromUnit(
			Unit unit) {

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

	public String getCodeline(
			String filepath) {

		int lastIndex = filepath.lastIndexOf('/');

		if (lastIndex != -1) {
			return filepath.substring(lastIndex + 1);
		} else {
			return "";
		}
	}

	public String loadResourceTemplate(
			String path)
			throws IOException {
		try (InputStream inputStream = TextHelper.class.getResourceAsStream(path);
				Scanner scanner = new Scanner(inputStream, StandardCharsets.UTF_8)) {
			scanner.useDelimiter("\\A");
			return scanner.hasNext() ? scanner.next() : "";
		}
	}
}
