# One opaque payload to or from the LoRa radio.
#
# Published on /lora/rx once per complete, CRC-validated frame received.
# Subscribed on /lora/tx to queue a payload for transmission.
#
# The bytes are opaque to the LoRa bridge: it frames, CRCs and queues them and
# nothing else. Interpreting them is the codec layer's job.
uint8[] data

# Transmit ordering class. Lower goes first.
#
# Set by the sender, which is the only party that knows what the payload means.
# The bridge sorts its outbound queue on this number and still never reads a
# payload byte, so it stays as opaque as it was. Ignored entirely on /lora/rx.
#
# Two classes, not a scale. URGENT is for the messages whose loss breaks task
# ownership -- an ACK resolves a retransmit campaign, and a GRANT is the only
# message sent reliably. Everything else is repeated by the coordinator on its
# own schedule, or is loss-tolerant by design.
#
# The default of 0 means an unset field reads as URGENT. That is the safe
# direction: a publisher that never heard of priority keeps the behaviour it
# had before this field existed, rather than being silently starved behind
# traffic that did set it.
uint8 PRIORITY_URGENT = 0
uint8 PRIORITY_BULK = 1
uint8 priority

# Link statistics for a received frame. Current firmware does not report these,
# so has_link_stats is false and rssi/snr must be ignored. Subscribers must
# tolerate their absence. Ignored entirely on /lora/tx.
bool has_link_stats
int16 rssi          # dBm
float32 snr         # dB
