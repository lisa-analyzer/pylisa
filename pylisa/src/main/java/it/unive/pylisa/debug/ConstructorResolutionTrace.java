package it.unive.pylisa.debug;

import java.util.ArrayList;
import java.util.List;

public final class ConstructorResolutionTrace {

	private static final List<ResolutionEvent> EVENTS = new ArrayList<>();

	private ConstructorResolutionTrace() {
	}

	public static synchronized void record(
			String site,
			String className,
			List<String> immediateAncestors,
			String attribute,
			String lookupPath,
			String runtimeTypes,
			String resolvedOwner,
			String mode) {
		EVENTS.add(new ResolutionEvent(site, className, new ArrayList<>(immediateAncestors), attribute, lookupPath,
				runtimeTypes, resolvedOwner, mode));
	}

	public static synchronized List<ResolutionEvent> snapshotAndClear() {
		List<ResolutionEvent> copy = new ArrayList<>(EVENTS);
		EVENTS.clear();
		return copy;
	}

	public record ResolutionEvent(
			String site,
			String className,
			List<String> immediateAncestors,
			String attribute,
			String lookupPath,
			String runtimeTypes,
			String resolvedOwner,
			String mode) {
	}
}
