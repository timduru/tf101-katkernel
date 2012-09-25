/*
 *  linux/fs/ext4/bitmap.c
 *
 * Copyright (C) 1992, 1993, 1994, 1995
 * Remy Card (card@masi.ibp.fr)
 * Laboratoire MASI - Institut Blaise Pascal
 * Universite Pierre et Marie Curie (Paris VI)
 */

#include <linux/buffer_head.h>
#include <linux/jbd2.h>
#include "ext4.h"

#ifdef EXT4FS_DEBUG

unsigned int ext4_count_free(struct buffer_head *map, unsigned int numchars)
{
	return numchars * BITS_PER_BYTE - memweight(map->b_data, numchars);
}

#endif  /*  EXT4FS_DEBUG  */

