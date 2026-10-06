package it.unive.pylisa.libraries.bytes;

import it.unive.lisa.program.cfg.CFG;
import it.unive.lisa.program.cfg.CodeLocation;
import it.unive.lisa.program.cfg.statement.Expression;
import it.unive.lisa.program.type.Int32Type;
import it.unive.pylisa.cfg.type.PyBytesType;
import it.unive.pylisa.libraries.strings.StrGetItemNative;
import it.unive.pylisa.symbolic.operators.bytes.BytesLength;
import it.unive.pylisa.symbolic.operators.bytes.BytesOperation;

/**
 * Native implementation of {@code bytes.__getitem__(self, index)}: {@code b[i]}
 * is an {@code int} (raising {@code IndexError} if {@code i} is out of range),
 * and {@code b[start:stop:step]} is {@code bytes} (raising {@code ValueError}
 * if {@code step} is zero). Any other index raises {@code TypeError}.
 */
public class BytesGetItem extends StrGetItemNative {

	protected BytesGetItem(
			CFG cfg,
			CodeLocation location,
			Expression[] params) {
		super(cfg, location, params, BytesLength.INSTANCE, BytesOperation.GETITEM, BytesOperation.GETSLICE,
				Int32Type.INSTANCE, PyBytesType.INSTANCE);
	}

	public static BytesGetItem build(
			CFG cfg,
			CodeLocation location,
			Expression[] exprs) {
		return new BytesGetItem(cfg, location, exprs);
	}
}
