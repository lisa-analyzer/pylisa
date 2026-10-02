package it.unive.pylisa;

import it.unive.lisa.program.language.validation.BaseValidationLogic;

/**
 * Python-specific program validation logic.
 * <p>
 * Currently delegates entirely to {@link BaseValidationLogic}. Since
 * {@link it.unive.pylisa.program.PyClassUnit} extends
 * {@link it.unive.lisa.program.ClassUnit}, the base dispatch in
 * {@link BaseValidationLogic#validateAndFinalize(it.unive.lisa.program.Unit)}
 * correctly routes Python class units through the full
 * {@code validateAndFinalize(ClassUnit)} validation path — no override is
 * required. This class is retained as an extension point for future
 * Python-specific validation rules.
 */
public class PythonValidationLogic extends BaseValidationLogic {
}
