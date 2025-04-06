# SCDTR

---

# Must do :

***LUX measurement:***

- [x] Implement a function that when invoked returns the
illuminance measured in LUX.

***LED driver:***

- [x] Implement a function that, when invoked, sets the power of
the led to some required value.

***System Identificationn:***

- [x] Implement a function that computes the static gains of the
system at initialisation.

***Luminaire PID controller:***

- [x] Implement a C++ class with functions to perform
feedback/feedforward control of a luminaire that can set its illuminance to a desired
value.

***Can-Bus communication protocol:***

- [x] Implement a network protocol where one node can send
messages to any other node and broadcast messages to all nodes. Characterise the
communication delays involved in your implementation.

***Embedded Concurrent Programming:***

- [ ] Implement a concurrent application on the microcontroller
with a control task, a Can-Bus communication task, a PC to Can-Bus message
routing mechanism (hub function), and a user interaction task. The concurrent
application must be of the non-blocking type, so no task can prevent other tasks
from executing.

***Network Initialisation:***

- [x] Implement a boot procedure in which the different nodes
recognise each other, calibrate the system gains, and synchronise to start operating
the distributed control system.

***Distributed Control:***

- [ ] Implement and present results of at least one of the distributed
solutions to the project presented in the course that require communication between
control nodes.
 
