// ta-build-stimulus-gen.c: Generate tANS table building stimulus and reference output for testbench.

// Useful function location: 
// ZSTD_entropyCompressSequences_internal: in zstd_compress.c. Entrance of the 
// entropy stage. Check it for how it encode the stream. 

// Similar to the Huffman tree stimulus generation, this file also use the raw
// Snappy stream output to feed into the ZSTD encoding function.

