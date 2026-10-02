package it.unive.pylisa.frontend;

import com.google.gson.Gson;
import com.google.gson.stream.JsonReader;
import java.io.FileReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.SortedMap;
import java.util.TreeMap;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

/**
 * Reads and normalizes Python source text from a file or Jupyter notebook.
 */
class SourceReader {

	private static final Logger log = LogManager.getLogger(SourceReader.class);

	private final String filePath;
	private final boolean notebook;
	private final List<Integer> cellOrder;

	SourceReader(
			String filePath,
			boolean notebook,
			List<Integer> cellOrder) {
		this.filePath = filePath;
		this.notebook = notebook;
		this.cellOrder = cellOrder;
	}

	/**
	 * Reads and normalizes the source text. For plain Python files the file is
	 * read directly; for notebooks the code cells are concatenated in
	 * {@link #cellOrder} order (or document order if empty).
	 */
	String readNormalizedSource() throws IOException {
		if (!notebook)
			return readNormalizedFile(filePath);

		Gson gson = new Gson();
		JsonReader reader = gson.newJsonReader(new FileReader(filePath));
		Map<?, ?> map = gson.fromJson(reader, Map.class);
		@SuppressWarnings("unchecked")
		List<Map<?, ?>> cells = (ArrayList<Map<?, ?>>) map.get("cells");
		SortedMap<Integer, String> codeBlocks = new TreeMap<>();
		for (int i = 0; i < cells.size(); i++) {
			Map<?, ?> cell = cells.get(i);
			String ctype = (String) cell.get("cell_type");
			if (ctype.equals("code")) {
				@SuppressWarnings("unchecked")
				List<String> codeList = (List<String>) cell.get("source");
				codeBlocks.put(i, transformToCode(codeList));
			}
		}

		StringBuilder code = new StringBuilder();
		if (cellOrder.isEmpty())
			for (Entry<Integer, String> c : codeBlocks.entrySet())
				code.append(c.getValue()).append("\n");
		else {
			log.warn("The following cells contain code and can be analyzed: " + codeBlocks.keySet());
			for (int idx : cellOrder) {
				String str = codeBlocks.get(idx);
				if (str == null)
					log.warn("Cell " + idx + " does not contain code and will be skipped");
				else
					code.append(str).append("\n");
			}
		}

		return code.toString();
	}

	/**
	 * Regex matching a Python numeric literal that may contain underscore
	 * separators (PEP 515, Python 3.6+). The negative lookbehind ensures we do
	 * not match a digit that is part of an identifier (e.g. {@code x1_000}).
	 */
	private static final Pattern UNDERSCORE_NUMBER = Pattern.compile("(?<![a-zA-Z_])\\d[\\d_]*");

	static String readNormalizedFile(
			String path)
			throws IOException {
		String source = Files.readString(Path.of(path), StandardCharsets.UTF_8);
		if (source.isEmpty() || source.charAt(source.length() - 1) != '\n')
			source = source + "\n";
		// Strip underscore separators from numeric literals so the ANTLR
		// grammar (which predates PEP 515) can parse them.
		Matcher m = UNDERSCORE_NUMBER.matcher(source);
		StringBuffer sb = new StringBuffer();
		while (m.find())
			m.appendReplacement(sb, m.group().replace("_", ""));
		m.appendTail(sb);
		return sb.toString();
	}

	private static String transformToCode(
			List<String> codeList) {
		return String.join("\n", codeList) + "\n";
	}
}
