package it.unive.pylisa.libraries.lists;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.pylisa.libraries.LibrarySpecificationProvider;

/**
 * Native implementation of {@code list.index(self, x, start, stop)}, the index
 * of the first element equal to {@code x}: an {@code int} between {@code 0} and
 * the length of the list (excluded), raising {@code ValueError} if {@code x}
 * might not be found, and {@code TypeError} if {@code start} or {@code stop}
 * might not be {@code int}s (or {@code None}). The elements are compared as by
 * {@link ListCount}, and any exception might be raised if they might be objects
 * (see {@link LibrarySpecificationProvider#UNKNOWN_EXCEPTION}).
 */
public class ListIndex extends ListCount {

	protected ListIndex(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, "index", true, params);
	}

	public static ListIndex build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new ListIndex(cfg, location, exprs);
	}
}
