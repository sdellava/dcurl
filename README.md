# dcurl - Multi-threaded Curl implementation
Hardware-accelerated implementation for IOTA PearlDiver, which utilizes multi-threaded SIMD and GPU - and FPGA.


# Differences to original dcurl library

IOTA PoW (proof of work) is one of the biggeste challenges for small microcontrollers - even for not so small controllers like an ARM-SoC which is used on Raspberry Pi.

This repository is about the implementation of PoW in hardware to boost PoW performance of small controllers.

Currently, it is running on a custom Raspberry Pi extension board with Cyclone 10 LP (25kLE @ 188MHz, 98% resources used) and archives 15.8MH/s - about 200x faster than a Raspberry without Hardware Accelerator. And about 5.8x faster than the SSE dcurl version on a quad-core i5.

# Updates

The project website https://microengineer.eu/2018/04/25/iota-pearl-diver-fpga/ is always up to date :)

All hardware and software will become open-source.

If you think, the project is worth supporting, please consider to leave me a tip at:

LLEYMHRKXWSPMGCMZFPKKTHSEMYJTNAZXSAYZGQUEXLXEEWPXUNWBFDWESOJVLHQHXOPQEYXGIRBYTLRWHMJAOSHUY

Discord: pmaxuw#8292

Thank you very much :)

<br><br>
<br><br>




