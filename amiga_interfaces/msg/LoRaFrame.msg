# One opaque payload to or from the LoRa radio.
#
# Published on /lora/rx once per complete, CRC-validated frame received.
# Subscribed on /lora/tx to queue a payload for transmission.
#
# The bytes are opaque to the LoRa bridge: it frames, CRCs and queues them and
# nothing else. Interpreting them is the codec layer's job.
uint8[] data

# Link statistics for a received frame. Current firmware does not report these,
# so has_link_stats is false and rssi/snr must be ignored. Subscribers must
# tolerate their absence. Ignored entirely on /lora/tx.
bool has_link_stats
int16 rssi          # dBm
float32 snr         # dB
