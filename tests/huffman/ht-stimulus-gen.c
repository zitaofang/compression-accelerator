// ht-stimulus-gen.c: Generate simulus for Huffman Tree module testbench

// Function prototype from trees.c
/* ===========================================================================
 * Allocate the match buffer, initialize the various tables and save the
 * location of the internal file attribute (ascii/binary) and method
 * (DEFLATE/STORE).
 */
void ct_init(unsigned short *attr, int *methodp);
/* ===========================================================================
 * Determine the best encoding for the current block: dynamic trees, static
 * trees or store, and output the encoded block to the zip file. This function
 * returns the total compressed length for the file so far.
 */
off_t flush_block(char *buf, unsigned long stored_len, int eof);
/* ===========================================================================
 * Save the match info and tally the frequency counts. Return true if
 * the current block must be flushed.
 */
int ct_tally (int dist, int lc)

// This program will take a prebuilt Huffman tree and a stream. 