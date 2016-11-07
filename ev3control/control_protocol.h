/*
 * ev3dev-mapping ev3control control protocol header file
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

#pragma once

#include <stdint.h> //uint16_t, int32_t

const int CONTROL_PROTOCOL_VERSION=1;

const int CONTROL_BUFFER_BYTES=512;
const int CONTROL_MAX_ATTRIBUTE_DATA_LENGTH=255; //uint8_t attribute length

enum ControlCommands {KEEPALIVE=0, ENABLE=1, DISABLE=2, DISABLE_ALL=3, ENABLED=-1, DISABLED=-2, FAILED=-3 };

// header
const int CONTROL_HEADER_BYTES=12;

struct control_header
{
	uint64_t timestamp_us;
	uint8_t protocol_version;
	int8_t command;
	uint16_t payload_length;
};


uint16_t GetControlHeaderPayloadLength(const char *header);
void GetControlHeader(const char *msg, control_header *header);

// attributes

enum ControlAttributes {CONTROL_ATTRIBUTES_FIRST=0, UNIQUE_NAME=0, CALL=1, CREATION_DELAY_MS=2, RETURN_VALUE=3, CONTROL_ATTRIBUTES_LAST=3};
const int CONTROL_ATTRIBUTES_LENGTHS[] = {0, 0, sizeof(uint16_t), sizeof(int32_t)};
const bool CONTROL_ATTRIBUTES_ZERO_TERMINATED[] = {true, true, false, false};

struct control_attribute
{
	uint8_t attribute;
	uint8_t length;
	const char *data;
};

bool ParseControlMessage(const control_header &header, const char *payload, control_attribute attributes[], int attributes_length,const ControlAttributes expected_attributes[]);

// attributes data

const char *GetControlAttributeString(const control_attribute &attribute);
uint16_t GetControlAttributeU16(const control_attribute &attribute);
int32_t GetControlAttributeI32(const control_attribute &attribute);

// message creation

bool PutControlHeader(char *buffer, int buffer_length, uint64_t timestamp_us, int8_t command);

//prerequisities:
// - PutControlHeader called with buffer argument
bool PutControlAttributeString(char *buffer, int buffer_length, uint8_t attribute, const char *data);

//prerequisities:
// - PutControlHeader called with buffer argument
bool PutControlAttributeI32(char *buffer, int buffer_length, uint8_t attribute, int32_t data);

//prerequisities:
// - PutControlHeader called with buffer argument
// - optionally PutControlAttributeXY any number of times
int GetControlMessageLength(char *buffer);