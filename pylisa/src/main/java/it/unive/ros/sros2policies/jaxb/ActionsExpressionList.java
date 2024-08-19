//
// This file was generated by the JavaTM Architecture for XML Binding(JAXB) Reference Implementation, v2.3.1 
// See <a href="https://javaee.github.io/jaxb-v2/">https://javaee.github.io/jaxb-v2/</a> 
// Any modifications to this file will be lost upon recompilation of the source schema. 
// Generated on: 2024.02.05 at 04:38:39 PM UTC 
//

package it.unive.ros.sros2policies.jaxb;

import jakarta.xml.bind.annotation.XmlAccessType;
import jakarta.xml.bind.annotation.XmlAccessorType;
import jakarta.xml.bind.annotation.XmlAttribute;
import jakarta.xml.bind.annotation.XmlElement;
import jakarta.xml.bind.annotation.XmlType;
import java.util.ArrayList;
import java.util.List;

/**
 * <p>
 * Java class for ActionsExpressionList complex type.
 * <p>
 * The following schema fragment specifies the expected content contained within
 * this class.
 * 
 * <pre>
 * &lt;complexType name="ActionsExpressionList"&gt;
 *   &lt;complexContent&gt;
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType"&gt;
 *       &lt;sequence maxOccurs="unbounded"&gt;
 *         &lt;element name="action" type="{}Expression"/&gt;
 *       &lt;/sequence&gt;
 *       &lt;attribute name="call" type="{}RuleQualifier" /&gt;
 *       &lt;attribute name="execute" type="{}RuleQualifier" /&gt;
 *     &lt;/restriction&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "ActionsExpressionList", propOrder = {
		"action"
})
public class ActionsExpressionList {

	@XmlElement(required = true)
	protected List<String> action;
	@XmlAttribute(name = "call")
	protected RuleQualifier call;
	@XmlAttribute(name = "execute")
	protected RuleQualifier execute;

	/**
	 * Gets the value of the action property.
	 * <p>
	 * This accessor method returns a reference to the live list, not a
	 * snapshot. Therefore any modification you make to the returned list will
	 * be present inside the JAXB object. This is why there is not a
	 * <CODE>set</CODE> method for the action property.
	 * <p>
	 * For example, to add a new item, do as follows:
	 * 
	 * <pre>
	 * getAction().add(newItem);
	 * </pre>
	 * <p>
	 * Objects of the following type(s) are allowed in the list {@link String }
	 */
	public List<String> getAction() {
		if (action == null) {
			action = new ArrayList<String>();
		}
		return this.action;
	}

	/**
	 * Gets the value of the call property.
	 * 
	 * @return possible object is {@link RuleQualifier }
	 */
	public RuleQualifier getCall() {
		return call;
	}

	/**
	 * Sets the value of the call property.
	 * 
	 * @param value allowed object is {@link RuleQualifier }
	 */
	public void setCall(
			RuleQualifier value) {
		this.call = value;
	}

	/**
	 * Gets the value of the execute property.
	 * 
	 * @return possible object is {@link RuleQualifier }
	 */
	public RuleQualifier getExecute() {
		return execute;
	}

	/**
	 * Sets the value of the execute property.
	 * 
	 * @param value allowed object is {@link RuleQualifier }
	 */
	public void setExecute(
			RuleQualifier value) {
		this.execute = value;
	}

}
