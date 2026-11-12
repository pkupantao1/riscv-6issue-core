/*
 * hostcompat.h - 编内核主机工具时强制包含 (scripts/build_linux.sh 的 KHOSTCFLAGS)
 * 132 的主机 glibc 是 2.17：没有 copy_file_range (2.27 起)。gen_init_cpio 调用失败时会退回 read/write，
 * 这里给一个总是返回 ENOSYS 的替身。
 */
#ifndef O6_HOSTCOMPAT_H
#define O6_HOSTCOMPAT_H
#include <sys/types.h>
#include <errno.h>
static inline ssize_t copy_file_range(int fd_in, long long *off_in, int fd_out, long long *off_out,
                                      size_t len, unsigned int flags)
{
	(void)fd_in; (void)off_in; (void)fd_out; (void)off_out; (void)len; (void)flags;
	errno = ENOSYS;
	return -1;
}
#endif
