"""Host regression for required persistence versus volatile business settings."""

import os
from pathlib import Path
import shutil
import subprocess
import tempfile

from test_ntn_control import BSP, function


STUBS = r"""
#include <assert.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
typedef uint8_t rt_uint8_t;
typedef uint16_t rt_uint16_t;
typedef uint32_t rt_uint32_t;
typedef uint64_t rt_uint64_t;
typedef uint32_t rt_tick_t;
typedef size_t rt_size_t;
typedef int rt_bool_t;
typedef int rt_err_t;
#define RT_TRUE 1
#define RT_FALSE 0
#define RT_EOK 0
#define RT_EINVAL 2
#define RT_EIO 3
#define RT_EFULL 4
#define RT_NULL NULL
#define RT_WAITING_FOREVER (-1)
#define rt_memset memset
#define rt_memcmp memcmp
#define rt_strncpy strncpy
#define rt_strncmp strncmp
#define rt_strlen strlen
#define LOG_E(...) ((void)0)
#define LOG_I(...) ((void)0)
struct rt_mutex { int held; };
static void rt_mutex_take(struct rt_mutex *lock, int wait)
{ (void)wait; assert(!lock->held); lock->held = 1; }
static void rt_mutex_release(struct rt_mutex *lock)
{ assert(lock->held); lock->held = 0; }
static unsigned char nv[8192];
static struct fal_partition { int present; } partition = {1};
static int flash_writable = 1;
static int rt_tick_get(void) { return 100; }
static int fal_partition_erase(const struct fal_partition *p, unsigned off, size_t n)
{ assert(p == &partition && off + n <= sizeof(nv)); if (!flash_writable) return -1; memset(nv + off, 0xFF, n); return (int)n; }
static int fal_partition_write(const struct fal_partition *p, unsigned off, const uint8_t *data, size_t n)
{ assert(p == &partition && off + n <= sizeof(nv)); memcpy(nv + off, data, n); return (int)n; }
static int fal_partition_read(const struct fal_partition *p, unsigned off, uint8_t *data, size_t n)
{ assert(p == &partition && off + n <= sizeof(nv)); memcpy(data, nv + off, n); return (int)n; }
"""

CASES = r"""
int main(void)
{
    uint64_t first;
    param_config_t saved;
    s_param_inited = RT_TRUE;
    param_set_defaults(&s_config);
    assert(s_config.work_mode == 0 && s_config.rs232_baud == 19200);
    assert(param_set_host("example.org", PARAM_SAVE_BEST_EFFORT) == PARAM_APPLIED_VOLATILE);
    assert(param_set_ports(8000, 0, PARAM_SAVE_BEST_EFFORT) == PARAM_APPLIED_VOLATILE);
    assert(param_set_work_mode(1, PARAM_SAVE_BEST_EFFORT) == PARAM_APPLIED_VOLATILE);
    assert(param_set_rs232_baud(115200, PARAM_SAVE_BEST_EFFORT) == PARAM_APPLIED_VOLATILE);
    assert(param_config_is_valid(&s_config) && !s_persisted_config);
    assert(s_config.work_mode == 1 && s_config.rs232_baud == 115200);
    assert(s_config.remote_port == 8000 && strcmp(s_config.host, "example.org") == 0);
    assert(param_set_work_mode(2, PARAM_SAVE_BEST_EFFORT) == -RT_EINVAL);
    assert(param_reserve_report_ids(1, &first) < 0 && s_config.next_report_id == 1);
    assert(param_reset_factory() < 0 && s_config.work_mode == 1);

    s_param_part = &partition;
    assert(param_set_work_mode(1, PARAM_SAVE_BEST_EFFORT) == RT_EOK);
    assert(s_persisted_config && s_config.revision == 1);
    assert(fal_partition_read(&partition, 4096, (uint8_t *)&saved, sizeof(saved)) == sizeof(saved));
    assert(param_config_is_valid(&saved) && saved.work_mode == 1);
    flash_writable = 0;
    assert(param_set_work_mode(0, PARAM_SAVE_BEST_EFFORT) == PARAM_APPLIED_VOLATILE);
    assert(!s_persisted_config && s_config.work_mode == 0 && s_config.revision == 1);
    assert(memcmp(&saved, nv + 4096, sizeof(saved)) == 0 && !s_param_lock.held);
    return 0;
}
"""


def main():
    compiler = os.environ.get("HOST_CC") or shutil.which("gcc")
    if not compiler:
        raise SystemExit("Set HOST_CC to the host GCC executable")
    header = (BSP / "applications/param_manager.h").read_text(encoding="utf-8")
    defines = "\n".join(line for line in header.splitlines() if line.startswith("#define PARAM_"))
    config = header[header.index("#pragma pack(1)"):header.index("/* Schema-3")]
    policy = header[header.index("typedef enum"):header.index("/* Successful application")]
    source = (BSP / "applications/param_manager.c").read_text(encoding="utf-8")
    statics = source[source.index("#define PARAM_SECTOR_SIZE"):source.index("static rt_uint32_t param_calc_crc")]
    functions = ("param_calc_crc", "param_set_defaults", "param_baud_is_valid", "param_ipv4_is_valid",
                 "param_host_is_valid", "param_config_is_valid", "param_save_locked", "param_finish_update",
                 "param_set_work_mode", "param_set_host", "param_set_ports", "param_set_rs232_baud",
                 "param_reserve_report_ids", "param_reset_factory")
    code = "\n".join((STUBS, defines, config, policy, statics,
                      *(function(source, name) for name in functions), CASES))
    with tempfile.TemporaryDirectory(prefix="ntn_param_storage_") as directory:
        path = Path(directory) / "test.c"
        binary = path.with_suffix(".exe")
        path.write_text(code, encoding="utf-8")
        subprocess.run([compiler, "-std=c99", "-Wall", "-Wextra", "-Werror", str(path), "-o", str(binary)], check=True)
        subprocess.run([str(binary)], check=True)


if __name__ == "__main__":
    main()
