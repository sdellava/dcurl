# dcurl - Multi-threaded Curl implementation
Hardware-accelerated implementation for IOTA PearlDiver, which utilizes multi-threaded SIMD and GPU - and FPGA.


# Differences to original dcurl library

IOTA PoW (proof of work) is one of the biggeste challenges for small microcontrollers - even for not so small controllers like an ARM-SoC which is used on Raspberry Pi.

This repository is about the implementation of PoW in hardware to boost PoW performance of small controllers.

Currently, it is running on Altera DE1 (quite old Cyclon2 with 22kLE @ 120MHz, 85% resources used) and archives 4.2MH/s - this enables a Raspberry Pi to perform PoW within 2-3 seconds (>x30 performance gain) :)

# Next steps

The next step is to develop and build an extension PCB which fits perfectly on top of a Raspi - I think, I'll need 8 weeks from now (=mid June'18).

All hardware and software will become open-source.

If you think, the project is worth supporting, please consider to leave me a tip at:

LLEYMHRKXWSPMGCMZFPKKTHSEMYJTNAZXSAYZGQUEXLXEEWPXUNWBFDWESOJVLHQHXOPQEYXGIRBYTLRWHMJAOSHUY

Discord: pmaxuw#8292

Thank you very much :)

<br><br>
<br><br>




