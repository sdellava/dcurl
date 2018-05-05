# dcurl - Multi-threaded Curl implementation
Hardware-accelerated implementation for IOTA PearlDiver, which utilizes multi-threaded SIMD and GPU - and FPGA.


# Differences to original dcurl library

IOTA PoW (proof of work) is one of the biggeste challenges for small microcontrollers - even for not so small controllers like an ARM-SoC which is used on Raspberry Pi.

This repository is about the implementation of PoW in hardware to boost PoW performance of small controllers.

Currently, it is running on Altera DE1 (quite old Cyclon2 with 22kLE @ 120MHz, 85% resources used) and archives 4.2MH/s - this enables a Raspberry Pi to perform PoW within 2-3 seconds (>x30 performance gain) :)

# Next steps

The next step is to develop and build an extension PCB which fits perfectly on top of a Raspi - I think, I'll need 8 weeks from now (=mid June'18).

All hardware and software will become open-source.
<!---
If you think, the project is worth supporting, please consider to leave me a tip at:

LLEYMHRKXWSPMGCMZFPKKTHSEMYJTNAZXSAYZGQUEXLXEEWPXUNWBFDWESOJVLHQHXOPQEYXGIRBYTLRWHMJAOSHUY

Thank you very much :)
-->
<br><br>
<br><br>








# Introduction
dcurl exploits SIMD instructions on CPU and OpenCL on GPU. Both CPU and GPU accelerations can be
enabled in multi-threaded execuction fashion, resulting in much faster proof-of-work (PoW) for IOTA
Reference Implementation (IRI).

# Warning
* You need to configure OpenCL platform and device by yourself in ```src/clcontext.c```
* You need to configure paths and flags of OpenCL installation in ```mk/opencl.mk```
* Check JDK installation and set JAVA_HOME if you wish to specify.
* Only one GPU can be facilitated with dcurl at the moment.
* If your platform doesn't support Intel SSE, dcurl would be compiled with naive implementation.

# Build Instructions
* dcurl allows various combinations of build configurations to fit final use scenarios.
* You can execute `make config` and then edit file `build/local.mk` for custom build options.
    - ``BUILD_AVX``: build Intel AVX-accelerated Curl backend.
    - ``BUILD_GPU``: build OpenCL-based GPU accelerations.
    - ``BUILD_FPGA``: build FPGA support (requires FPGA board connected to your Raspberry Pi).
    - ``BUILD_JNI``: build a shared library for IRI. The build system would generate JNI header file
                   from downloading from
                   [latest JAVA source](https://github.com/chenwei-tw/iri/tree/feat/new_pow_interface).
    - ``BUILD_COMPAT``: build extra cCurl compatible interface.
* Alternatively, you can specify conditional build as following:
```shell
$ make BUILD_GPU=0 BUILD_JNI=1 BUILD_AVX=1
```
# FPGA requirement
* You need to download and install the bcm2835 library
```
$ wget http://www.airspayce.com/mikem/bcm2835/bcm2835-1.55.tar.gz
$ tar zxvf bcm2835-1.55.tar.gz
$ cd bcm2835-1.xx
$ ./configure
$ make
$ sudo make check
$ sudo make install
```

* You then need to enable SPI on your Raspberry Pi using
```
$ sudo raspi-config
-> Interfacing Options
-> SPI
-> Enable
$ sudo reboot

```

# Test
* Test with GPU
```shell
$ make BUILD_GPU=1 check
```

* Expected Results
```
*** Validating build/test-trinary ***
        [ Verified ]
*** Validating build/test-curl ***
        [ Verified ]
*** Validating build/test-multi_pow_cpu ***
        [ Verified ]
*** Validating build/test-pow_sse ***
        [ Verified ]
*** Validating build/test-pow_cl ***
        [ Verified ]
*** Validating build/test-multi_pow_gpu ***
        [ Verified ]
```

* Test with AVX but no GPU
```shell
$ make BUILD_AVX=1 check
```

* Expected Results
```
*** Validating build/test-trinary ***
        [ Verified ]
*** Validating build/test-curl ***
        [ Verified ]
*** Validating build/test-multi_pow_cpu ***
        [ Verified ]
*** Validating build/test-pow_avx ***
        [ Verified ]
```

# Tweaks
* ```dcurl_init(2, 1)``` in ```jni/iri-pearldiver-exlib.c```
    * ```2``` means 2 pow tasks executed in CPU,
    * ```1``` means 1 pow tasks executed in GPU at the same time.
* Number of threads to find nonce in CPU
    * ```$ export DCURL_NUM_CPU=26```

# Performance 
After integrating dcurl into IRI, performance of <```attachToTangle```> is measured as following.
* Each sampling is measured with 30 transaction trytes and total 200 samples are measured.
* mwm = 14, 26 CPU threads to find nonce
* Settings: enable 2 pow tasks in CPU, 1 pow tasks in GPU at the same time
![](https://lh4.googleusercontent.com/2U_TpfAtEbPdHBcGKD1zl0t0bzo2Rubj0DxXxvV-Rh31Yr7oCCtptutQpLLizMgR7ousEXUtwM6RASnQLOJnGePhQ5Emh1w8l8GlKzMtZ0Yv-TySF2gh3u48BAmllAJv2VjNaxgFGCA)

# IRI Adaptation
[Modified IRI accepting external PoW Library](https://github.com/chenwei-tw/iri/tree/feat/new_pow_interface)
* ```$ cd ~/iri && mvn compile && mvn package```
* ```$ cp ~/dcurl/build/libdcurl.so ~/iri```
* ```$ cd ~/iri && java -Djava.library.path=./ -jar target/iri.jar -p <port> --pearldiver-exlib dcurl```

# IOTA PoW Node
[gagathos/iota-gpu-pow](https://github.com/gagathos/iota-gpu-pow)
* You can construct a IOTA PoW node, which uses `ccurl` by default
* Generate a drop-in replacement for `ccurl` and acquire performance boost!
    * ```$ make BUILD_COMPAT=1 check```
    * ```$ cp ./build/libdcurl.so <iota-gpu-pow>/libccurl.so```

# TODO
* ~~More test program :(~~
* ~~Pre-compile OpenCL kernel functions and include it in dcurl.~~
* Automatically configure dcurl after init()

# Licensing

`dcurl` is freely redistributable under the MIT License.
Use of this source code is governed by a MIT-style license that can be
found in the `LICENSE` file.
