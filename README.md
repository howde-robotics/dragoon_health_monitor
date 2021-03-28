# Health Monitor


This is the watchdog that will monitor sensor heartbeats and alert users to any issues. 

There are several launch files in here right now. Anything with "faux" in front of it is a test of the health monitor node. These faux nodes simulate heartbeats from the various systems that the monitor is watching. 

TODO: 
1. Decide what happens with the output of the HM. Does it go to the visualizer? Or a rover executive?
2. Test with real sensor nodes 

Here is an architecture for the system:

![DHM Architecture Diagram](doc/DHMA.jpg "Architecture Diagram")
