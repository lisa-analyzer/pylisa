//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.1 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2023.09.22 at 03:06:47 PM CEST 
//


package it.unive.ros.permissions.jaxb;


import jakarta.xml.bind.annotation.XmlEnum;
import jakarta.xml.bind.annotation.XmlType;

/**
 * <p>Java class for DefaultAction.
 * 
 * <p>The following schema fragment specifies the expected content contained within this class.
 * <p>
 * <pre>
 * &lt;simpleType name="DefaultAction"&gt;
 *   &lt;restriction base="{http://www.w3.org/2001/XMLSchema}string"&gt;
 *     &lt;enumeration value="ALLOW"/&gt;
 *     &lt;enumeration value="DENY"/&gt;
 *   &lt;/restriction&gt;
 * &lt;/simpleType&gt;
 * </pre>
 * 
 */
@XmlType(name = "DefaultAction")
@XmlEnum
public enum DefaultAction {

    ALLOW,
    DENY;

    public String value() {
        return name();
    }

    public static DefaultAction fromValue(String v) {
        return valueOf(v);
    }

}