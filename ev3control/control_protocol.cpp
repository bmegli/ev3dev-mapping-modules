/*
 * ev3dev-mapping ev3control control protocol implementation file
 *
 * Copyright (C) 2016 Bartosz Meglicki <meglickib@gmail.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 3 as
 * published by the Free Software Foundation.
 * This program is distributed "as is" WITHOUT ANY WARRANTY of any
 * kind, whether express or implied; without even the implied warranty
 * of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/*
 *  For high level protocol overview see:
 *  https://github.com/bmegli/ev3dev-mapping/issues/5
 * 
 */

#include "control_protocol.h"

#include <endian.h> //htobe16, htobe32, htobe64, be16toh, be32toh, be64toh
#include <string.h> //strlen

// header
const int CONTROL_HEADER_TIMESTAMP_OFFSET=0;
const int CONTROL_HEADER_PROTOCOL_VERSION_OFFSET=8;
const int CONTROL_HEADER_COMMAND_OFFSET=9;
const int CONTROL_HEADER_PAYLOAD_LENGTH_OFFSET=10;

//attributes
const int CONTROL_ATTRIBUTE_HEADER_BYTES=2;
const int CONTROL_ATTRIBUTE_OFFSET=0;
const int CONTROL_ATTRIBUTE_LENGTH_OFFSET=1;
const int CONTROL_ATTRIBUTE_DATA_OFFSET=2;

uint16_t GetControlHeaderPayloadLength(const char *header)
{
	uint16_t payload;
	memcpy(&payload, header+CONTROL_HEADER_PAYLOAD_LENGTH_OFFSET, 2);
	return be16toh(payload);
}

void SetControlHeaderPayloadLength(char *buffer, uint16_t payload_length)
{
	payload_length=htobe16(payload_length);
	memcpy(buffer + CONTROL_HEADER_PAYLOAD_LENGTH_OFFSET, &payload_length, 2);
}

void GetControlHeader(const char *msg, control_header *header)
{
	memcpy(&header->timestamp_us, msg+CONTROL_HEADER_TIMESTAMP_OFFSET, 8);
	header->timestamp_us = be64toh(header->timestamp_us);

	header->protocol_version = *((uint8_t*) (msg+CONTROL_HEADER_PROTOCOL_VERSION_OFFSET) );	
	header->command = *((int8_t*) (msg+CONTROL_HEADER_COMMAND_OFFSET) );
	
	memcpy(&header->payload_length, msg+CONTROL_HEADER_PAYLOAD_LENGTH_OFFSET, 2);
	header->payload_length = be16toh(header->payload_length);
}

int GetControlAttribute(const char *payload, control_attribute *attribute)
{
	attribute->attribute = *((uint8_t*) (payload+CONTROL_ATTRIBUTE_OFFSET) );
	attribute->length = *((uint8_t*) (payload+CONTROL_ATTRIBUTE_LENGTH_OFFSET) );
	attribute->data = payload + CONTROL_ATTRIBUTE_DATA_OFFSET;
	return attribute->length+CONTROL_ATTRIBUTE_HEADER_BYTES;
}

bool ParseControlMessage(const control_header &header, const char *message_payload, control_attribute attributes[], int attributes_length,const ControlAttributes expected_attributes[])
{
	char *payload=(char*)message_payload;
	const char *message_boundrary=payload+header.payload_length;
	
	for(int i=0;i< attributes_length;++i)
	{
		//check if attribute header fits message
		if(payload + CONTROL_ATTRIBUTE_HEADER_BYTES > message_boundrary)
			return false;
		
		payload += GetControlAttribute(payload, attributes +i);
		
		const control_attribute &a=attributes[i];
		
		//check if attribute is as expected
		if(a.attribute != expected_attributes[i])
			return false;
		
		//check if attribute data fits message
		if(a.data + a.length > message_boundrary)
			return false;
		
		//check if attribute has data
		if(a.length == 0)
			return false;
		
		//if attribute is known
		if(a.attribute >= CONTROL_ATTRIBUTES_FIRST && a.attribute <= CONTROL_ATTRIBUTES_LAST)
		{
			//check if attribute size matches known sizes
			if(CONTROL_ATTRIBUTES_LENGTHS[a.attribute] && a.length != CONTROL_ATTRIBUTES_LENGTHS[a.attribute])
				return false;
			
			//check if zero terminated attribute is terminated
			if(CONTROL_ATTRIBUTES_ZERO_TERMINATED[a.attribute] && a.data[a.length-1]!='\0')
				return false;
		}
	}
	
	return true;
}

const char *GetControlAttributeString(const control_attribute &attribute)
{
	return attribute.data;
}
uint16_t GetControlAttributeU16(const control_attribute &attribute)
{
	uint16_t value;
	memcpy(&value, attribute.data, 2);
	return be16toh(value);
}
int32_t GetControlAttributeI32(const control_attribute &attribute)
{
	int32_t value;
	memcpy(&value, attribute.data, 4);
	return be32toh(value);
}

// message creation

bool PutControlHeader(char *buffer, int buffer_length, uint64_t timestamp_us, int8_t command)
{
	if(buffer_length < CONTROL_HEADER_BYTES)
		return false;
	
	timestamp_us = htobe64(timestamp_us);
	memcpy(buffer + CONTROL_HEADER_TIMESTAMP_OFFSET, &timestamp_us, 8);	
	
	*((uint8_t*) (buffer + CONTROL_HEADER_PROTOCOL_VERSION_OFFSET) ) = CONTROL_PROTOCOL_VERSION;

	*((int8_t*) (buffer + CONTROL_HEADER_COMMAND_OFFSET) ) = command;

	memset(buffer + CONTROL_HEADER_PAYLOAD_LENGTH_OFFSET, 0, 2);	
				
	return true;
}


int PutControlAttribute(char *buffer, int buffer_length, uint8_t attribute, int attribute_length)
{
	//check if attribute length will attribute limits
	if(attribute_length > CONTROL_MAX_ATTRIBUTE_DATA_LENGTH)
		return -1;
		
	int payload_length=GetControlHeaderPayloadLength(buffer);
	
	//check if attribute will fit buffer
	if(CONTROL_HEADER_BYTES + payload_length + CONTROL_ATTRIBUTE_HEADER_BYTES + attribute_length > buffer_length)
		return -1;
		
	char *attr_data=buffer + CONTROL_HEADER_BYTES + payload_length;
	*((uint8_t*) (attr_data + CONTROL_ATTRIBUTE_OFFSET) ) = attribute;
	*((uint8_t*) (attr_data + CONTROL_ATTRIBUTE_LENGTH_OFFSET) ) = attribute_length;
	
	SetControlHeaderPayloadLength(buffer, payload_length + CONTROL_ATTRIBUTE_HEADER_BYTES + attribute_length);
	
	return CONTROL_HEADER_BYTES + payload_length + CONTROL_ATTRIBUTE_DATA_OFFSET;
}

bool PutControlAttributeString(char *buffer, int buffer_length, uint8_t attribute, const char *data)
{
	int len= strlen(data) + 1;
	
	int attribute_data_offset=PutControlAttribute(buffer, buffer_length, attribute, len);
	
	if(attribute_data_offset==-1)
		return false;
		
	memcpy(buffer + attribute_data_offset, data, len);
		
	return true;
}

bool PutControlAttributeI32(char *buffer, int buffer_length, uint8_t attribute, int32_t data)
{
	int len = sizeof(int32_t);

	int attribute_data_offset=PutControlAttribute(buffer, buffer_length, attribute, len);
	
	if(attribute_data_offset==-1)
		return false;
	data = htobe32(data);
	memcpy(buffer + attribute_data_offset, &data, 4);
	
	return true;
}

int GetControlMessageLength(char *buffer)
{
	return CONTROL_HEADER_BYTES + GetControlHeaderPayloadLength(buffer);
}