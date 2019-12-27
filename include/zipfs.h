/*
 * Copyright (c) 2014 Century Systems, Co., Ltd.
 *
 * under GPLv2 only
 */

#ifndef __ZIPFS_H__
#define __ZIPFS_H__

#include <linux/types.h>
#include <linux/list.h>
#include <linux/zlib.h>
#include <ioctl.h>

#define COMP_NONE    0x0000
#define COMP_DEFLATE 0x0008

struct zip_file_header {
	uint8_t signature[4];	/* [50 4b 03 04] */
	uint16_t min_ver;
	uint16_t flags;
	uint16_t comp;
	uint16_t last_mod_time;
	uint16_t last_mod_date;
	uint32_t crc32;
	uint32_t comp_size;
	uint32_t uncomp_size;
	uint16_t filename_len;
	uint16_t extra_fields_len;
	uint8_t data[0];
} __attribute__ ((packed));

struct zip_central_header {
	uint8_t signature[4];	/* [50 4b 01 02] */
	uint16_t creat_ver;
	uint16_t min_ver;
	uint16_t flags;
	uint16_t comp;
	uint16_t last_mod_time;
	uint16_t last_mod_date;
	uint32_t crc32;
	uint32_t comp_size;
	uint32_t uncomp_size;
	uint16_t filename_len;
	uint16_t extfield_len;
	uint16_t comment_len;
	uint16_t disk_no;
	uint16_t prop_int;
	uint32_t prop_ext;
	uint32_t file_header_offset;
	uint8_t data[0];
} __attribute__ ((packed));

struct zip_central_header_last {
	uint8_t signature[4];	/* [50 4b 05 06] */
	uint16_t n_disks;
	uint16_t disk_no;
	uint16_t n_records_disk;
	uint16_t n_records;
	uint32_t central_dir_size;
	uint32_t central_dir_offset;
	uint16_t comment_len;
	uint8_t data[0];
} __attribute__ ((packed));

struct zipfs_handle_data {
	char *name;	/* filename */
	uint32_t size;	/* file size */
	uint16_t comp;	/* compression method */

	struct zip_file_header header;

	int fd;
	size_t offset;		/* offset in the image */
	size_t data_pos;	/* data head offset */
	size_t pos;		/* pos in the data */

	z_stream *zs;
	uint8_t *zbuf;
	size_t zbuflen;

	struct list_head list;
};

struct zipfs_handle {
	size_t size;	/* zipfile size */

	struct zip_central_header header;
	
	int nb_data_entries;
	char *filename;

	struct list_head list;
};

#endif /* __ZIPFS_H__ */
