"""Contract net itself. No ROS, no clock of its own, no network.

    coordinator.py  the state machine, both roles at once
    auction.py      the owner side: announce, collect, arbitrate
    bidding.py      the bidder side: fitness, backoff, suppression
    registry.py     who else is out there, and whether they still are

``CoordinatorSession`` takes its clock as an argument and its transport as a
port, so every timing rule in here -- backoff windows, auction deadlines, peer
expiry -- is exercised by advancing a number rather than by waiting. That is
the property that makes the acceptance suite a claim about the whole layer
instead of a smoke test.

Every robot runs one, in both roles simultaneously: owner of tasks it may have
to shed, bidder on tasks its peers announce.
"""
