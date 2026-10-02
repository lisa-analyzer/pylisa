package it.unive.pylisa.cfg.statement;

import it.unive.lisa.type.Type;
import it.unive.pylisa.program.type.NoInfoType;
import java.util.Set;

public final class UnknownSymbolUtils {

	private UnknownSymbolUtils() {
	}

	public static String unknownAttributeName(
			String ownerName,
			String memberName) {
		String owner = ownerName == null ? "<unknown-owner>" : ownerName;
		if (owner.startsWith("$"))
			owner = owner.substring(1);
		return "$" + owner + "::" + memberName;
	}

	public static boolean isUnresolvedTypeSet(
			Set<Type> runtimeTypes) {
		if (runtimeTypes == null || runtimeTypes.isEmpty())
			return true;
		return runtimeTypes.stream().allMatch(type -> type == NoInfoType.INSTANCE);
	}
}
