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
 * Java class for ServicesExpressionList complex type.
 * <p>
 * The following schema fragment specifies the expected content contained within
 * this class.
 * 
 * <pre>
 * &lt;complexType name="ServicesExpressionList"&gt;
 *   &lt;complexContent&gt;
 *     &lt;restriction base="{http://www.w3.org/2001/XMLSchema}anyType"&gt;
 *       &lt;sequence maxOccurs="unbounded"&gt;
 *         &lt;element name="service" type="{}Expression"/&gt;
 *       &lt;/sequence&gt;
 *       &lt;attribute name="reply" type="{}RuleQualifier" /&gt;
 *       &lt;attribute name="request" type="{}RuleQualifier" /&gt;
 *     &lt;/restriction&gt;
 *   &lt;/complexContent&gt;
 * &lt;/complexType&gt;
 * </pre>
 */
@XmlAccessorType(XmlAccessType.FIELD)
@XmlType(name = "ServicesExpressionList", propOrder = {
		"service"
})
public class ServicesExpressionList {

	@XmlElement(required = true)
	protected List<String> service;
	@XmlAttribute(name = "reply")
	protected RuleQualifier reply;
	@XmlAttribute(name = "request")
	protected RuleQualifier request;

	/**
	 * Gets the value of the service property.
	 * <p>
	 * This accessor method returns a reference to the live list, not a
	 * snapshot. Therefore any modification you make to the returned list will
	 * be present inside the JAXB object. This is why there is not a
	 * <CODE>set</CODE> method for the service property.
	 * <p>
	 * For example, to add a new item, do as follows:
	 * 
	 * <pre>
	 * getService().add(newItem);
	 * </pre>
	 * <p>
	 * Objects of the following type(s) are allowed in the list {@link String }
	 */
	public List<String> getService() {
		if (service == null) {
			service = new ArrayList<String>();
		}
		return this.service;
	}

	/**
	 * Gets the value of the reply property.
	 * 
	 * @return possible object is {@link RuleQualifier }
	 */
	public RuleQualifier getReply() {
		return reply;
	}

	/**
	 * Sets the value of the reply property.
	 * 
	 * @param value allowed object is {@link RuleQualifier }
	 */
	public void setReply(
			RuleQualifier value) {
		this.reply = value;
	}

	/**
	 * Gets the value of the request property.
	 * 
	 * @return possible object is {@link RuleQualifier }
	 */
	public RuleQualifier getRequest() {
		return request;
	}

	/**
	 * Sets the value of the request property.
	 * 
	 * @param value allowed object is {@link RuleQualifier }
	 */
	public void setRequest(
			RuleQualifier value) {
		this.request = value;
	}

}