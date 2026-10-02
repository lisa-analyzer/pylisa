package it.unive.pylisa.libraries.natives;

import java.lang.annotation.Documented;
import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Declares that the callable a {@link LibraryNative} models may never return
 * for some input, as a blocking call does: its model may then leave no
 * continuation for a reachable input. The declaration belongs to the callable,
 * so readers of the analysis results can check it on the model's class.
 */
@Documented
@Retention(RetentionPolicy.RUNTIME)
@Target(ElementType.TYPE)
public @interface Diverges {
}
