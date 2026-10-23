// SPDX-License-Identifier: MIT
#include "management_protocol.h"
#include <assert.h>
#include <initializer_list>
#include <stdio.h>
#include <math.h>
using namespace amov_device;

static void requests()
{
    Request request;
    request.operation = Operation::ACTIVATE;
    request.transaction = 0x12345678;
    request.boot = 0x90abcdef;
    request.revision = 0x11223344;
    const uint8_t uid[6] = {0, 0x12, 0x34, 0x56, 0x78, 0x9a};
    memcpy(request.uid, uid, sizeof(uid));
    request.value_length = 21;
    request.value[0] = 1;
    memcpy(request.value + 1, "00000000000000000042", 20);
    uint8_t wire[REQUEST_SIZE + 1];
    memset(wire, 0xcc, sizeof(wire));
    assert(encode_request(request, wire, REQUEST_SIZE));
    assert(wire[REQUEST_SIZE] == 0xcc);
    const uint8_t prefix[] = {1, 6, 0, 0, 0x78, 0x56, 0x34, 0x12,
        0xef, 0xcd, 0xab, 0x90, 0, 0x12, 0x34, 0x56, 0x78, 0x9a, 0, 0,
        0x44, 0x33, 0x22, 0x11, 21, 0, 0, 0};
    assert(memcmp(wire, prefix, sizeof(prefix)) == 0);
    assert(memcmp(wire + 45, "00000000000000000042", 20) == 0);
    Request decoded;
    assert(decode_request(wire, REQUEST_SIZE, decoded));
    assert(decoded.operation == Operation::ACTIVATE && decoded.transaction == request.transaction);
    assert(decoded.boot == request.boot && decoded.revision == request.revision);
    assert(decoded.value_length == 21 && memcmp(decoded.value, request.value, 21) == 0);
    for (size_t length = 0; length < REQUEST_SIZE; ++length) {
        assert(!decode_request(wire, length, decoded));
    }
    assert(!decode_request(wire, REQUEST_SIZE + 1, decoded));
    for (const size_t offset : {size_t(3), size_t(25), size_t(26), size_t(27), size_t(107)}) {
        wire[offset] = 1;
        assert(!decode_request(wire, REQUEST_SIZE, decoded));
        wire[offset] = 0;
    }
    wire[24] = 65;
    assert(!decode_request(wire, REQUEST_SIZE, decoded));
    wire[24] = 21;
    wire[0] = 2;
    assert(!decode_request(wire, REQUEST_SIZE, decoded));
    request.transaction = 0;
    assert(!encode_request(request, wire, sizeof(wire)));
    request.transaction = 1;
    assert(!encode_request(request, wire, REQUEST_SIZE - 1));
    assert(!encode_request(request, nullptr, REQUEST_SIZE));
}

static void maximum_strings()
{
    Request request;
    request.operation = Operation::SET;
    request.transaction = 1;
    request.type = Type::CHAR64;
    memcpy(request.name, "ABCDEFGHIJKLMNOP", 16);
    request.value_length = 64;
    for (unsigned i = 0; i < 64; ++i) { request.value[i] = uint8_t('A' + i % 26); }
    uint8_t wire[PAYLOAD_SIZE];
    assert(encode_request(request, wire, sizeof(wire)));
    Request decoded;
    assert(decode_request(wire, REQUEST_SIZE, decoded));
    assert(decoded.name[16] == 0 && memcmp(decoded.name, request.name, 16) == 0);
    assert(decoded.value_length == 64 && memcmp(decoded.value, request.value, 64) == 0);
    request.name[4] = 0; // Non-canonical names must not alias a different parameter.
    assert(!encode_request(request, wire, sizeof(wire)));
}

static void responses()
{
    Response response;
    response.operation = Operation::METADATA;
    response.transaction = 0xffffffff;
    response.boot = 42;
    response.type = Type::CHAR64;
    response.result = Result::SAVED;
    response.index = 46;
    response.total = 47;
    response.flags = WRITABLE | REBOOT_REQUIRED | DEFAULT_AVAILABLE | DURABLE;
    response.value_length = 64;
    response.max_length = 64;
    response.default_length = 64;
    response.activation_state = 1;
    response.revision = 4;
    response.minimum = -1.5f;
    response.maximum = 65535.0f;
    memcpy(response.name, "PUBLIC_KEY1", 11);
    for (unsigned i = 0; i < 64; ++i) {
        response.value[i] = uint8_t(i + 32);
        response.default_value[i] = uint8_t(128 + i);
    }
    uint8_t wire[RESPONSE_SIZE + 1];
    wire[RESPONSE_SIZE] = 0xa5;
    assert(encode_response(response, wire, RESPONSE_SIZE));
    assert(wire[RESPONSE_SIZE] == 0xa5);
    assert(wire[18] == 46 && wire[20] == 47 && wire[22] == 101);
    assert(wire[114] == 0xc0 && wire[115] == 0xbf); // -1.5 IEEE754 little endian
    Response decoded;
    assert(decode_response(wire, RESPONSE_SIZE, decoded));
    assert(decoded.minimum == -1.5f && decoded.maximum == 65535.0f);
    assert(decoded.transaction == response.transaction && decoded.flags == response.flags);
    assert(decoded.value_length == 64 && decoded.default_length == 64);
    assert(memcmp(decoded.value, response.value, 64) == 0);
    assert(memcmp(decoded.default_value, response.default_value, 64) == 0);
    for (size_t length = 0; length < RESPONSE_SIZE; ++length) {
        assert(!decode_response(wire, length, decoded));
    }
    for (size_t i = 185; i < RESPONSE_SIZE; ++i) {
        wire[i] = 1;
        assert(!decode_response(wire, RESPONSE_SIZE, decoded));
        wire[i] = 0;
    }
    wire[120] = 65;
    assert(!decode_response(wire, RESPONSE_SIZE, decoded));
    wire[120] = 64;
    wire[23] = 1;
    assert(!decode_response(wire, RESPONSE_SIZE, decoded));
    assert(!decode_response(nullptr, RESPONSE_SIZE, decoded));
    assert(!encode_response(response, wire, RESPONSE_SIZE - 1));
}

static void operation_shapes()
{
    Request request;
    request.transaction = 42;
    uint8_t wire[REQUEST_SIZE];
    assert(encode_request(request, wire, sizeof(wire)));
    request.revision = 1;
    assert(!encode_request(request, wire, sizeof(wire))); // INFO is not an activation mutation.
    request.revision = 0;
    request.operation = Operation::METADATA;
    request.index = 46;
    assert(encode_request(request, wire, sizeof(wire)));
    request.operation = Operation::SET;
    request.type = Type::UINT8;
    strcpy(request.name, "CAN_NODE");
    request.value_length = 1;
    request.value[0] = 12;
    assert(!encode_request(request, wire, sizeof(wire))); // SET addresses a name, never a page index.
    request.index = 0;
    assert(encode_request(request, wire, sizeof(wire)));
    wire[20] = 1;
    Request decoded;
    assert(!decode_request(wire, sizeof(wire), decoded));
    request = Request{};
    request.transaction = 42;
    request.operation = Operation::STATUS;
    assert(!encode_request(request, wire, sizeof(wire))); // ID alone can alias another client's write.
    request.value_length = 32;
    memset(request.value, 0xab, request.value_length);
    assert(encode_request(request, wire, sizeof(wire)));
    request.value_length = 31;
    assert(!encode_request(request, wire, sizeof(wire)));
    request.value_length = 0;
    request.operation = Operation::RESET;
    assert(encode_request(request, wire, sizeof(wire)));
    request.operation = Operation::REBOOT;
    assert(encode_request(request, wire, sizeof(wire)));
    request.operation = Operation::GET;
    assert(!encode_request(request, wire, sizeof(wire)));
    strcpy(request.name, "RID_ACT_STATE");
    assert(encode_request(request, wire, sizeof(wire)));
}

static void numeric_values()
{
    uint8_t value[4];
    write_u32(value, 0xfedcba98);
    assert(value[0] == 0x98 && value[3] == 0xfe && read_u32(value) == 0xfedcba98);
    write_float(value, -123.25f);
    assert(read_float(value) == -123.25f);
    write_float(value, INFINITY);
    assert(isinf(read_float(value))); // Business validation, not codec, rejects non-finite parameter values.
    write_u16(value, 0x1234);
    assert(value[0] == 0x34 && value[1] == 0x12 && read_u16(value) == 0x1234);
}

static void factory_provisioning()
{
    Request request;
    request.operation = Operation::PROVISION;
    request.transaction = 19;
    request.boot = 42;
    request.value_length = 21;
    request.value[0] = 4;
    memcpy(request.value + 1, "AMOV0123456789012345", 20);
    uint8_t wire[REQUEST_SIZE];
    assert(encode_request(request, wire, sizeof(wire)));
    assert(wire[1] == 9 && wire[24] == 21 && wire[44] == 4);
    Request decoded;
    assert(decode_request(wire, sizeof(wire), decoded));
    assert(decoded.operation == Operation::PROVISION && decoded.value[0] == 4);
    assert(!memcmp(decoded.value + 1, request.value + 1, 20));
    for (const size_t offset : {size_t(2), size_t(18), size_t(20), size_t(28)}) {
        wire[offset] = 1;
        assert(!decode_request(wire, sizeof(wire), decoded));
        wire[offset] = 0;
    }
    wire[44] = 5;
    assert(!decode_request(wire, sizeof(wire), decoded));
    request.value_length = 20;
    assert(!encode_request(request, wire, sizeof(wire)));

    Response info;
    info.transaction = 19;
    info.default_length = 1;
    info.default_value[0] = FACTORY_PROVISION_CAPABLE | FACTORY_PROVISION_AVAILABLE;
    uint8_t reply[RESPONSE_SIZE];
    assert(encode_response(info, reply, sizeof(reply)));
    Response restored;
    assert(decode_response(reply, sizeof(reply), restored));
    assert(restored.flags == 0 && restored.default_length == 1 && restored.default_value[0] == 3);
}

int main()
{
    requests(); maximum_strings(); responses(); operation_shapes(); numeric_values(); factory_provisioning();
    puts("Management codec: golden offsets, bounded decoding, 20/64-byte values and numerics passed");
}
