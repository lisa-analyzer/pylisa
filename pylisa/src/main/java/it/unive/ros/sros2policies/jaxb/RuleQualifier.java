//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.1 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2024.02.05 at 04:38:39 PM UTC 
//

package it.unive.ros.sros2policies.jaxb;

import jakarta.xml.bind.annotation.XmlEnum;
import jakarta.xml.bind.annotation.XmlType;

/**
 * <p>
 * Java class for RuleQualifier.
 * <p>
 * The following schema fragment specifies the expected content contained within
 * this class.
 * <p>
 * 
 * <pre>
 * &lt;simpleType name="RuleQualifier"&gt;
 *   &lt;restriction base="{http://www.w3.org/2001/XMLSchema}string"&gt;
 *     &lt;enumeration value="ALLOW"/&gt;
 *     &lt;enumeration value="DENY"/&gt;
 *   &lt;/restriction&gt;
 * &lt;/simpleType&gt;
 * </pre>
 */
@XmlType(name = "RuleQualifier")
@XmlEnum
public enum RuleQualifier {

	ALLOW,
	DENY;

	public String value() {
		return name();
	}

	public static RuleQualifier fromValue(
			String v) {
		return valueOf(v);
	}

}
