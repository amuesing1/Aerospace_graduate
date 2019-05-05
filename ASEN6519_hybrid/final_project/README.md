Code is in Python 2.7, if running python 3 make sure to specify 2.7 before running.

Required packages:
scipy
numpy
matplotlib
yaml

Run main simulation:
$cd /src/
$python controller.py

This will bring up a simulation and graph the sub and its path in real time. This initializes the hybrid automaton contined in hybrid_automaton.py. The product of the system is taken and the reachability is also performed.

Run automaton code alone:

$python hybrid_automaton.py

This will print out the sequence of states and their locations to reach an accepting state.
