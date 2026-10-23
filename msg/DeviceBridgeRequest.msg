# Opaque MAVLink-to-DroneCAN service request. No application semantics.
uint64 timestamp
uint64 route_generation
uint64 session_id
uint32 request_id
uint8 mavlink_instance
uint8 source_system
uint8 source_component
uint8 protocol_version
uint8 operation
uint8 node_id
uint8 payload_length
uint8[192] payload

uint8 PROTOCOL_VERSION = 1
uint8 OPERATION_LIST_NODES = 0
uint8 OPERATION_TRANSFER = 1
uint8 MAX_PAYLOAD_LENGTH = 192
uint8 ORB_QUEUE_LENGTH = 8
