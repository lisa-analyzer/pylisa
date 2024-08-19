package it.unive.ros.sros2policies.jaxb;

import jakarta.xml.bind.JAXBContext;
import jakarta.xml.bind.JAXBElement;
import jakarta.xml.bind.JAXBException;
import jakarta.xml.bind.Marshaller;
import jakarta.xml.bind.Unmarshaller;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileNotFoundException;
import java.io.FileOutputStream;
import java.io.IOException;
import java.io.StringWriter;

public class JAXBROS2PermissionsHelpers {

	private static final JAXBContext jaxbContext;

	static {
		try {
			jaxbContext = JAXBContext.newInstance(Policy.class);
		} catch (JAXBException e) {
			throw new RuntimeException(e);
		}
	}

	public static Policy load(
			String fileName)
			throws JAXBException,
			FileNotFoundException {
		Unmarshaller jaxbUnmarshaller = jaxbContext.createUnmarshaller();
		JAXBElement<Policy> permissions = (JAXBElement<Policy>) jaxbUnmarshaller
				.unmarshal(new FileInputStream(fileName));
		return permissions.getValue();
	}

	public static void store(
			Policy policy,
			String fileName)
			throws JAXBException,
			IOException {
		Marshaller jaxbMarshaller = jaxbContext.createMarshaller();
		jaxbMarshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);
		File file = new File(fileName);
		file.getParentFile().mkdirs();
		file.createNewFile();
		jaxbMarshaller.marshal(new ObjectFactory().createPolicy(policy), new FileOutputStream(file));
	}

	public static String toString(
			Policy policy)
			throws JAXBException,
			IOException {
		Marshaller jaxbMarshaller = jaxbContext.createMarshaller();
		jaxbMarshaller.setProperty(Marshaller.JAXB_FORMATTED_OUTPUT, true);
		StringWriter sw = new StringWriter();
		jaxbMarshaller.marshal(new ObjectFactory().createPolicy(policy), sw);
		return sw.toString();
	}
}
