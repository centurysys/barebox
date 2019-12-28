/*
 * Copyright (c) 2014,2019 Century Systems, Co., Ltd.
 *
 * under GPLv2 ONLY
 */

#include <common.h>
#include <driver.h>
#include <fs.h>
#include <errno.h>
#include <fcntl.h>
#include <fs.h>
#include <malloc.h>
#include <init.h>
#include <linux/stat.h>
#include <linux/err.h>
#include <zipfs.h>
#include <libbb.h>
#include <rtc.h>

static struct zipfs_handle_data *zipfs_get_by_name(struct zipfs_handle *handle,
						   const char *name)
{
	struct zipfs_handle_data *d;

	if (!name)
		return NULL;

	list_for_each_entry(d, &handle->list, list) {
		if (strcmp(d->name, name) == 0)
			return d;
	}

	return NULL;
}

static int zipfs_open(struct device_d *dev, FILE *file, const char *filename)
{
	int ret;
	struct zipfs_handle *priv = dev->priv;
	struct zipfs_handle_data *d;
	struct zip_file_header header;

	if (filename[0] == '/')
		filename++;

	d = zipfs_get_by_name(priv, filename);
	if (!d)
		return -EINVAL;

	d->fd = open(priv->filename, O_RDONLY);
	if (d->fd < 0)
		return d->fd;

	/* parse zip local file header */
	ret = lseek(d->fd, d->offset, SEEK_SET);

	read(d->fd, (char *) &header, sizeof(struct zip_file_header));

	if (be32_to_cpu(*(uint32_t *) header.signature) != 0x504b0304) {
		printf("%s: signature error! (0x%08x)\n", __FUNCTION__,
		       be32_to_cpu(*(uint32_t *) header.signature));
		return -EINVAL;
	}

	if (d->comp == COMP_DEFLATE) {
		d->zbuflen = 1 * 1024 * 1024;
		d->zbuf = xzalloc(d->zbuflen);
		d->zs = xzalloc(sizeof(struct z_stream_s));
		d->zs->workspace = xzalloc(zlib_inflate_workspacesize());

		zlib_inflateInit2(d->zs, -15);
	}

	/* seek to head of data */
	ret = lseek(d->fd, header.filename_len + header.extra_fields_len, SEEK_CUR);
	d->data_pos = d->pos = ret;

	file->size = header.uncomp_size;
	file->priv = d;

	return 0;
}

static int zipfs_close(struct device_d *dev, FILE *file)
{
	struct zipfs_handle_data *d = file->priv;

	close(d->fd);

	if (d->comp == COMP_DEFLATE) {
		zlib_inflateEnd(d->zs);
		free(d->zbuf);
		free(d->zs);

		d->zbuf = NULL;
		d->zs = NULL;
	}

	return 0;
}

static int __zip_read_inflate(struct zipfs_handle_data *d, void *buf, size_t insize)
{
	struct z_stream_s *zs = d->zs;
	int readlen, ret;

	if (zs->avail_in == 0) {
		readlen = read(d->fd, d->zbuf, d->zbuflen);
		if (readlen <= 0)
			return 0;

		zs->next_in = d->zbuf;
		zs->avail_in = readlen;
	}

	zs->next_out = buf;
	zs->avail_out = insize;

	ret = zlib_inflate(zs, 0);

	if (ret == Z_OK || ret == Z_STREAM_END)
		ret = insize - zs->avail_out;
	else
		ret = -1;

	return ret;
}

static int zipfs_read(struct device_d *dev, FILE *file, void *buf, size_t insize)
{
	struct zipfs_handle_data *d = file->priv;

	if (d->comp == COMP_DEFLATE) {
		return __zip_read_inflate(d, buf, insize);
	} else {
		return read(d->fd, buf, insize);
	}
}

static int zipfs_lseek(struct device_d *dev, FILE *file, loff_t pos)
{
	struct zipfs_handle_data *d = file->priv;

	if (d->comp == COMP_DEFLATE) {
		/* todo */
	} else {
		lseek(d->fd, d->data_pos + pos, SEEK_SET);
		d->pos = pos;
	}

	return pos;
}

static DIR *zipfs_opendir(struct device_d *dev, const char *pathname)
{
	struct zipfs_handle *priv = dev->priv;
	DIR *dir;

	dir = xzalloc(sizeof(DIR));

	if (list_empty(&priv->list))
		return dir;

	dir->priv = list_first_entry(&priv->list,
				     struct zipfs_handle_data, list);

	return dir;
}

static struct dirent *zipfs_readdir(struct device_d *dev, DIR *dir)
{
	struct zipfs_handle *priv = dev->priv;
	struct zipfs_handle_data *d = dir->priv;

	if (!d || &d->list == &priv->list)
		return NULL;

	strcpy(dir->d.d_name, d->name);
	dir->priv = list_entry(d->list.next, struct zipfs_handle_data, list);
	return &dir->d;
}

static int zipfs_closedir(struct device_d *dev, DIR *dir)
{
	free(dir);
	return 0;
}

static int zipfs_stat(struct device_d *dev, const char *filename, struct stat *s)
{
	struct zipfs_handle *priv = dev->priv;
	struct zipfs_handle_data *d;

	if (filename[0] == '/')
		filename++;

	d = zipfs_get_by_name(priv, filename);
	if (!d)
		return -EINVAL;

	s->st_size = d->size;
	s->st_mode = S_IFREG | S_IRUSR | S_IRGRP | S_IROTH;

	return 0;
}

static void zipfs_remove(struct device_d *dev)
{
	struct zipfs_handle *priv = dev->priv;
	struct zipfs_handle_data *d, *tmp;

	list_for_each_entry_safe(d, tmp, &priv->list, list) {
		free(d->name);
		free(d);
	}

	free(priv->filename);
	free(priv);
}

static int __search_zip_entries(struct zipfs_handle *priv, int fd,
				struct zip_central_header_last *header)
{
	int i, ret = -1;
	struct zip_central_header cheader;
	struct zipfs_handle_data *entry;
	loff_t offset;
	uint16_t comp;

	offset = header->central_dir_offset;

	lseek(fd, offset, SEEK_SET);

	for (i = 0; i < header->n_records; i++) {
		ret = read(fd, (char *) &cheader, sizeof(cheader));

		if (be32_to_cpu(*(uint32_t *) cheader.signature) != 0x504b0102) {
			printf("%s: header signature error!\n", __FUNCTION__);
			goto err_out;
		}

		comp = le16_to_cpu(cheader.comp);
		/* check compression method ('stored' or 'deflate' are supported.) */
		if (comp != COMP_NONE && comp != COMP_DEFLATE) {
			printf("%s: compression method[0x%04x] not supported.\n",
			       __FUNCTION__, comp);
		} else {
			entry = xzalloc(sizeof(struct zipfs_handle_data));

			entry->name = xzalloc(cheader.filename_len + 1);
			read(fd, entry->name, cheader.filename_len);

			entry->size = cheader.uncomp_size;
			entry->offset = cheader.file_header_offset;
			entry->comp = comp;

			list_add_tail(&entry->list, &priv->list);
		}

		lseek(fd, cheader.extfield_len + cheader.comment_len, SEEK_CUR);
	}

	ret = 0;

err_out:
	return ret;
}

/*
 * open a zip file. This will check the header contents and
 * return a handle to the zipfile.
 */
static int __zip_open(struct zipfs_handle *priv)
{
	int fd, ret;
	struct zip_central_header_last header;
	struct stat st;
	size_t filesize, offset;

	fd = open(priv->filename, O_RDONLY);
	if (fd < 0) {
		printf("could not open: %s\n", errno_str());
		return fd;
	}

	ret = stat(priv->filename, &st);
	if (ret < 0) {
		printf("could not stat(), %s\n", errno_str());
		goto err_out;
	}

	filesize = st.st_size;
	offset = filesize - sizeof(struct zip_central_header_last);
	lseek(fd, offset, SEEK_SET);

	ret = read(fd, &header, sizeof(header));
	if (ret < 0) {
		printf("could not read: %s\n", errno_str());
		goto err_out;
	}

	if (be32_to_cpu(*((uint32_t *) header.signature)) != 0x504b0506) {
		printf("not zip file.\n");
		goto err_out;
	}

	priv->size = filesize;

	ret = __search_zip_entries(priv, fd, &header);
err_out:

	close(fd);

	return ret;
}

static int zipfs_probe(struct device_d *dev)
{
	struct fs_device_d *fsdev = dev_to_fs_device(dev);
	struct zipfs_handle *priv;
	int ret = 0, len;

	priv = xzalloc(sizeof(struct zipfs_handle));
	INIT_LIST_HEAD(&priv->list);
	dev->priv = priv;

	if (fsdev->backingstore[0] == '/') {
		/* full-path */
		len = strlen(fsdev->backingstore);
		priv->filename = xzalloc(len + 1);
		strncpy(priv->filename, fsdev->backingstore, len);
	} else {
		/* relative path */
		const char *cwd = getcwd();

		len = strlen(cwd) + strlen(fsdev->backingstore);
		priv->filename = xzalloc(len + 2); /* '/' + '\0' */
		sprintf(priv->filename, "%s/%s", cwd, fsdev->backingstore);
	}

	dev_dbg(dev, "mount: %s\n", priv->filename);

	ret = __zip_open(priv);
	if (ret)
		goto err;

	return 0;

err:
	zipfs_remove(dev);

	return ret;
}

static struct fs_driver_d zipfs_driver = {
	.open      = zipfs_open,
	.close     = zipfs_close,
	.read      = zipfs_read,
	.lseek     = zipfs_lseek,
	.opendir   = zipfs_opendir,
	.readdir   = zipfs_readdir,
	.closedir  = zipfs_closedir,
	.stat      = zipfs_stat,
	.flags     = 0,
	.type = filetype_uimage,
	.drv = {
		.probe  = zipfs_probe,
		.remove = zipfs_remove,
		.name = "zipfs",
	}
};

static int zipfs_init(void)
{
	return register_fs_driver(&zipfs_driver);
}
coredevice_initcall(zipfs_init);
