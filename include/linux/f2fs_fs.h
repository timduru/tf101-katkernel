/**
 * include/linux/f2fs_fs.h
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *             http://www.samsung.com/
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */
#ifndef _LINUX_F2FS_FS_H
#define _LINUX_F2FS_FS_H

#include <linux/pagemap.h>
#include <linux/types.h>

#define F2FS_SUPER_MAGIC	0xF2F52010
#define F2FS_SUPER_OFFSET	1		/* start sector # for sb */
#define F2FS_BLKSIZE		4096

#define NULL_ADDR		0x0U
#define NEW_ADDR		-1U

#define F2FS_ROOT_INO(sbi)	(sbi->root_ino_num)
#define F2FS_NODE_INO(sbi)	(sbi->node_ino_num)
#define F2FS_META_INO(sbi)	(sbi->meta_ino_num)

#define GFP_F2FS_MOVABLE	(__GFP_WAIT | __GFP_IO | __GFP_ZERO)

/*
 * For superblock
 */
struct f2fs_super_block {
	__le32 magic;		/* Magic Number */
	__le16 major_ver;	/* Major Version */
	__le16 minor_ver;	/* Minor Version */
	__le32 log_sectorsize;	/* log2 (Sector size in bytes) */
	__le32 log_sectors_per_block;	/* log2 (Number of sectors per block */
	__le32 log_blocksize;	/* log2 (Block size in bytes) */
	__le32 log_blocks_per_seg; /* log2 (Number of blocks per segment) */
	__le32 log_segs_per_sec; /* log2 (Number of segments per section) */
	__le32 secs_per_zone; /* Number of sections per zone */
	__le32 checksum_offset;	/* Checksum position in this super block */
	__le64 block_count;	/* Total number of blocks */
	__le32 section_count;	/* Total number of sections */
	__le32 segment_count;	/* Total number of segments */
	__le32 segment_count_ckpt; /* Total number of segments
				      in Checkpoint area */
	__le32 segment_count_sit; /* Total number of segments
				     in Segment information table */
	__le32 segment_count_nat; /* Total number of segments
				     in Node address table */
	/*Total number of segments in Segment summary area */
	__le32 segment_count_ssa;
	/* Total number of segments in Main area */
	__le32 segment_count_main;
	__le32 failure_safe_block_distance;
	__le64 segment0_blkaddr;	/* Start block address of Segment 0 */
	__le64 start_segment_checkpoint; /* Start block address of ckpt */
	__le64 sit_blkaddr;	/* Start block address of SIT */
	__le64 nat_blkaddr;	/* Start block address of NAT */
	__le64 ssa_blkaddr;     /* Start block address of SSA */
	__le64 main_blkaddr;	/* Start block address of Main area */
	__le32 root_ino;	/* Root directory inode number */
	__le32 node_ino;	/* node inode number */
	__le32 meta_ino;	/* meta inode number */
	__le32 volume_serial_number;	/* VSN is optional field */
	__le16 volume_name[8];	/* Volume Name. 8 unicode characters */
} __packed;

/*
 * For checkpoint
 */
struct f2fs_checkpoint {
	__le64 checkpoint_ver;		/* Checkpoint block version number */
	__le64 user_block_count;	/* Total number of blocks
					   in Main area excluding the number of
					   reserved blocks */
	__le64 valid_block_count;	/* Total number of valid blocks
					   in Main area */
	__le32 rsvd_segment_count;	/* Total number of reserved segments
					   (for garbage collection) */
	__le32 overprov_segment_count;	/* Total number of overprovision
					   segments */
	__le32 free_segment_count;	/* Total number of free segments
					   in Main area */
	__le32 bad_segment_count;	/* Total number of bad segments
					   in Main area */
	__le32 cur_node_segno[3];	/* Segment number of current node
					   segment */
	__le16 cur_node_blkoff[3];	/* Current node block offset
					   in the node segment */
	__le16 nat_upd_blkoff[3];	/* Block offset in current node segment
					   where the last NAT update happened */
	__le32 cur_data_segno[3];	/* Segment number of current log
					   segment */
	__le16 cur_data_blkoff[3];	/* Current data block offset
					   in the data segment */
	__le32 ckpt_flags;		/* Flags : umount and orphan_present */
	__le32 cp_pack_total_block_count;
	__le32 cp_pack_start_sum;	/* start block number of data summary */
	__le32 valid_node_count;	/* Total number of valid nodes */
	__le32 valid_inode_count;	/* Total number of valid inodes */
	__le32 next_free_nid;		/* Next free node number */
	__le32 sit_ver_bitmap_bytesize;	/* Default value 64 */
	__le32 nat_ver_bitmap_bytesize; /* Default value 256 */
	__le32 checksum_offset;		/* Checksum position
					   in this checkpoint block */
	__le64 elapsed_time;		/* elapsed time while partition
					   is mounted */
	unsigned char alloc_type[6];	/* allocation type of current segment */

	/* SIT and NAT version bitmap */
	unsigned char sit_nat_version_bitmap[1];
} __packed;

/*
 * For orphan inode management
 */
#define F2FS_ORPHANS_PER_BLOCK	1020

struct f2fs_orphan_block {
	__le32 ino[F2FS_ORPHANS_PER_BLOCK];	/* inode numbers */
	__le32 reserved;
	__le16 blk_addr;	/* block index in current CP */
	__le16 blk_count;	/* Number of orphan inode blocks in CP */
	__le32 entry_count;	/* Total number of orphan nodes in current CP */
	__le32 check_sum;	/* CRC32 for orphan inode block */
} __packed;

/*
 * For NODE structure
 */
struct f2fs_extent {
	__le32 fofs;
	__le32 blk_addr;
	__le32 len;
} __packed;

#define F2FS_MAX_NAME_LEN	256
#define ADDRS_PER_INODE         929	/* Address Pointers in an Inode */
#define ADDRS_PER_BLOCK         1018	/* Address Pointers in a Direct Block */
#define NIDS_PER_BLOCK          1018	/* Node IDs in an Indirect Block */

struct f2fs_inode {
	__le16 i_mode;			/* File mode */
	__le16 i_reserved;		/* Reserved */
	__le32 i_uid;			/* User ID */
	__le32 i_gid;			/* Group ID */
	__le32 i_links;			/* Links count */
	__le64 i_size;			/* File size in bytes */
	__le64 i_blocks;		/* File size in bytes */
	__le32 i_atime;			/* Access time */
	__le32 i_ctime;			/* inode Change time */
	__le32 i_mtime;			/* Modification time */
	__le32 i_btime;			/* file creation time*/
	__le32 current_depth;
	__le32 i_xattr_nid;		/* nid to save xattr */
	__le32 i_flags;			/* file attributes */
	__le32 i_pino;			/* parent inode number */
	__le32 i_namelen;		/* file name length */
	__u8 i_name[F2FS_MAX_NAME_LEN];	/* file name for SPOR */

	struct f2fs_extent i_ext;	/* caching a largest extent */

	__le32 i_addr[ADDRS_PER_INODE];	/* Pointers to data blocks */

	__le32 i_nid[5];		/* direct(2), indirect(2),
						double_indirect(1) node id */
} __packed;

struct direct_node {
	__le32 addr[ADDRS_PER_BLOCK];	/* aray of data block address */
} __packed;

struct indirect_node {
	__le32 nid[NIDS_PER_BLOCK];	/* aray of data block address */
} __packed;

struct node_footer {
	__le32 nid;		/* node id */
	__le32 ino;		/* inode nunmber */
	__le32 cold:1;		/* cold mark */
	__le32 fsync:1;		/* fsync mark */
	__le32 dentry:1;	/* dentry mark */
	__le32 offset:29;	/* offset in inode's node space */
	__le64 cp_ver;		/* checkpoint version */
	__le32 next_blkaddr;	/* next node page block address */
} __packed;

struct f2fs_node {
	union {
		struct f2fs_inode i;
		struct direct_node dn;
		struct indirect_node in;
	};
	struct node_footer footer;
} __packed;

/*
 * For NAT entries
 */
#define NAT_ENTRY_PER_BLOCK (PAGE_CACHE_SIZE / sizeof(struct f2fs_nat_entry))

struct f2fs_nat_entry {
	__u8 version;
	__le32 ino;
	__le32 block_addr;
} __packed;

struct f2fs_nat_block {
	struct f2fs_nat_entry entries[NAT_ENTRY_PER_BLOCK];
} __packed;

/*
 * For SIT entries
 */
#define SIT_VBLOCK_MAP_SIZE 64
#define SIT_ENTRY_PER_BLOCK (PAGE_CACHE_SIZE / sizeof(struct f2fs_sit_entry))

struct f2fs_sit_entry {
	__le16 vblocks;
	__u8 valid_map[SIT_VBLOCK_MAP_SIZE];
	__le64 mtime;
} __packed;

struct f2fs_sit_block {
	struct f2fs_sit_entry entries[SIT_ENTRY_PER_BLOCK];
} __packed;

/**
 * For segment summary
 *
 * NOTE : For initializing fields, you must use set_summary
 *
 * - If data page, nid represents dnode's nid
 * - If node page, nid represents the node page's nid.
 *
 * The ofs_in_node is used by only data page. It represents offset
 * from node's page's beginning to get a data block address.
 * ex) data_blkaddr = (block_t)(nodepage_start_address + ofs_in_node)
 */
struct f2fs_summary {
	__le32 nid;
	union {
		__u8 reserved[3];
		struct {
			__u8 version;
			__le16 ofs_in_node;
		} __packed;
	};
} __packed;

struct summary_footer {
	unsigned char entry_type;
	__u32 check_sum;
} __packed;

#define	SUMMARY_SIZE		(sizeof(struct f2fs_summary))
#define	SUM_FOOTER_SIZE		(sizeof(struct summary_footer))
#define ENTRIES_IN_SUM		512
#define SUM_ENTRY_SIZE		(SUMMARY_SIZE * ENTRIES_IN_SUM)
#define SUM_JOURNAL_SIZE	(PAGE_CACHE_SIZE - SUM_FOOTER_SIZE -\
				SUM_ENTRY_SIZE)
struct nat_journal_entry {
	__le32 nid;
	struct f2fs_nat_entry ne;
} __packed;

struct sit_journal_entry {
	__le32 segno;
	struct f2fs_sit_entry se;
} __packed;

#define NAT_JOURNAL_ENTRIES	((SUM_JOURNAL_SIZE - 2) /\
				sizeof(struct nat_journal_entry))
#define NAT_JOURNAL_RESERVED	((SUM_JOURNAL_SIZE - 2) %\
				sizeof(struct nat_journal_entry))
#define SIT_JOURNAL_ENTRIES	((SUM_JOURNAL_SIZE - 2) /\
				sizeof(struct sit_journal_entry))
#define SIT_JOURNAL_RESERVED	((SUM_JOURNAL_SIZE - 2) %\
				sizeof(struct sit_journal_entry))
enum {
	NAT_JOURNAL = 0,
	SIT_JOURNAL
};

struct nat_journal {
	struct nat_journal_entry entries[NAT_JOURNAL_ENTRIES];
	__u8 reserved[NAT_JOURNAL_RESERVED];
} __packed;

struct sit_journal {
	struct sit_journal_entry entries[SIT_JOURNAL_ENTRIES];
	__u8 reserved[SIT_JOURNAL_RESERVED];
} __packed;

struct f2fs_summary_block {
	struct f2fs_summary entries[ENTRIES_IN_SUM];
	union {
		__le16 n_nats;
		__le16 n_sits;
	};
	union {
		struct nat_journal nat_j;
		struct sit_journal sit_j;
	};
	struct summary_footer footer;
} __packed;

/*
 * For directory operations
 */
#define F2FS_DOT_HASH		0
#define F2FS_DDOT_HASH		F2FS_DOT_HASH
#define F2FS_MAX_HASH		(~((0x3ULL) << 62))
#define F2FS_HASH_COL_BIT	((0x1ULL) << 63)

typedef __le32	f2fs_hash_t;

#define F2FS_NAME_LEN		8
#define NR_DENTRY_IN_BLOCK	214 /* the number of dentry in a block */
#define MAX_DIR_HASH_DEPTH	63 /* MAX level for dir lookup */

#define SIZE_OF_DIR_ENTRY	11	/* by byte */
#define SIZE_OF_DENTRY_BITMAP	((NR_DENTRY_IN_BLOCK + BITS_PER_BYTE - 1) / \
					BITS_PER_BYTE)
#define SIZE_OF_RESERVED	(PAGE_SIZE - ((SIZE_OF_DIR_ENTRY + \
				F2FS_NAME_LEN) * \
				NR_DENTRY_IN_BLOCK + SIZE_OF_DENTRY_BITMAP))

struct f2fs_dir_entry {
	__le32 hash_code;	/* hash code of file name */
	__le32 ino;		/* node number of inode */
	__le16 name_len;	/* the size of file name
				   length in unicode characters */
	__u8 file_type;
} __packed;

struct f2fs_dentry_block {
	__u8 dentry_bitmap[SIZE_OF_DENTRY_BITMAP];
	__u8 reserved[SIZE_OF_RESERVED];
	struct f2fs_dir_entry dentry[NR_DENTRY_IN_BLOCK];
	__u8 filename[NR_DENTRY_IN_BLOCK][F2FS_NAME_LEN];
} __packed;

enum {
	F2FS_FT_UNKNOWN,
	F2FS_FT_REG_FILE,
	F2FS_FT_DIR,
	F2FS_FT_CHRDEV,
	F2FS_FT_BLKDEV,
	F2FS_FT_FIFO,
	F2FS_FT_SOCK,
	F2FS_FT_SYMLINK,
	F2FS_FT_MAX
};

#endif  /* _LINUX_F2FS_FS_H */
