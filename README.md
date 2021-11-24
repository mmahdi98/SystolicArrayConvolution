# SystolicArrayConvolution

* Original Work is Done by H. T. Kung et al, [here](https://apps.dtic.mil/sti/citations/ADA104872).
* You can see an overview of the architecture in [file](3x3%20kernel%20for%20convolution.pdf').
* The Original PE is implemented in serial bitwise style which is in [serial_bitwise_pe.v](serial_bitwise_pe.v) file.
* The eight bit parallel or let's say 8bit serial version of PE, which uses a barrel shifter instead of a serial shifter, is in [serial_8bit_pe.v](serial_8bit_pe.v) file.
* [3x3 kernel](3x3_kernel.v), that is implemented in an 8bit approach.
* [5x5 kernel](5X5_kernel.v), 8bit too.
* [7x7 kernel](7x7_kernel.v), 8bit too.


