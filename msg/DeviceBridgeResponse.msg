# Transport result; a successful response does not interpret the device payload.
uint64 timestamp
uint64 request_timestamp
uint64 route_generation
uint64 session_id
uint32 request_id
uint8 mavlink_instance
uint8 target_system
uint8 target_component
uint8 operation
uint8 node_id
uint8 status
uint8 payload_length
uint8[192] payload

uint8 STATUS_OK = 0
uint8 STATUS_UNSUPPORTED = 1
uint8 STATUS_INVALID = 2
uint8 STATUS_BUSY = 3
uint8 STATUS_TIMEOUT = 4
uint8 STATUS_UNAVAILABLE = 5
uint8 ORB_QUEUE_LENGTH = 8
