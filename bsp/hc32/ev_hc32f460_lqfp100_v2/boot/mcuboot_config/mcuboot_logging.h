/* SPDX-License-Identifier: Apache-2.0 */
#ifndef NTN_MCUBOOT_LOGGING_H
#define NTN_MCUBOOT_LOGGING_H
void ntn_ota_log_error(const char *format, ...);
#define MCUBOOT_LOG_MODULE_REGISTER(...)
#define MCUBOOT_LOG_MODULE_DECLARE(...)
#define MCUBOOT_LOG_ERR(...) ntn_ota_log_error(__VA_ARGS__)
#define MCUBOOT_LOG_WRN(...) ((void)0)
#define MCUBOOT_LOG_INF(...) ((void)0)
#define MCUBOOT_LOG_DBG(...) ((void)0)
#endif
