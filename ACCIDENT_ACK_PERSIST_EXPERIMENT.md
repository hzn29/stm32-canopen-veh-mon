# Accident ACK and persistence experiment

## Frame format

B sends CANopen TPDO3 with COB-ID `0x382` and eight Classic CAN bytes:

| Byte(s) | Field |
| --- | --- |
| 0 | Event type: `1` collision, `2` rollover |
| 1 | Event flags |
| 2..5 | Full 32-bit little-endian event ID |
| 6 | Peak acceleration / 100 mg, saturated at 255 |
| 7 | Peak angular rate / 10 dps, saturated at 255 |

A returns COB-ID `0x502` with four little-endian bytes containing the same event ID.

## Retry behavior

B stores the event before transmission. It retries after 500 ms until an ACK is received. After ten unsuccessful retries the interval changes to 5 s. A duplicate event is acknowledged but only processed once.

## Power-loss behavior

The pending event, full event ID, peak values, event type, and retry count are stored in a two-page append journal. Each record contains a sequence number and CRC32. On boot the newest valid record is restored; a Flash page is erased only when the active page is full.

## Test procedure

1. Trigger an event on B and verify `0x382` and `0x502` in PCAN-View.
2. Disconnect A, verify B retries and enters the 5 s degraded interval after ten retries.
3. Reset B while the event is pending and verify that it resumes transmission with the same 32-bit event ID.
4. Reconnect A and verify one ACK clears the pending record.
