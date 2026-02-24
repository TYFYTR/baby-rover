# Architecture decision records (ADRs): design choices, tradeoffs, and rationale.

## Encoder reading — polling vs interrupts
- lgpio 0.2.0.0 callbacks broken on Pi 5
- Decision: poll at 100Hz via ROS 2 timer
- Tradeoff: may miss pulses at high speed
- Revisit: upgrade lgpio or switch library when moving to university rover