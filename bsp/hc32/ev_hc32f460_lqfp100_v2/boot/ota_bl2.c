/* Copyright (c) 2026, youyoutech.
 * SPDX-License-Identifier: Apache-2.0
 */
#include <stdbool.h>
#include <string.h>
#include <rtthread.h>
#include "ota_layout.h"
#include "ota_jump.h"
#include "ota_mailbox.h"
#include "ota_serial.h"
#include "ota_watchdog.h"
#include "app_flash.h"
#include "bootutil/bootutil.h"
#include "bootutil/boot_hooks.h"
#include "bootutil/boot_public_hooks.h"
#include "bootutil/image.h"
#include "bootutil_priv.h"
#include "mcuboot_config/mcuboot_logging.h"
#include "tinycrypt/ecc_platform_specific.h"
#include "flash_map_backend/flash_map_backend.h"

void ntn_ota_mcuboot_watchdog_feed(void)
{
    ntn_ota_feed_watchdog();
}

static bool compatible(const struct flash_area *area, const struct image_header *header)
{
    uint32_t descriptor[16];
    uint32_t vector[2];
    uint32_t entry;

    if (header->ih_magic != IMAGE_MAGIC || header->ih_hdr_size != NTN_OTA_HEADER_SIZE ||
        header->ih_flags != 0 || header->ih_img_size > NTN_OTA_APP_MAX ||
        header->ih_img_size < NTN_OTA_DESCRIPTOR_OFF + sizeof(descriptor) ||
        flash_area_read(area, NTN_OTA_HEADER_SIZE, vector, sizeof(vector)) ||
        flash_area_read(area, NTN_OTA_HEADER_SIZE + NTN_OTA_DESCRIPTOR_OFF,
                        descriptor, sizeof(descriptor)))
        return false;

    entry = vector[1] & ~1UL;
    return !memcmp(descriptor, "NTHC", 4) && descriptor[1] == 0x00400001 &&
           descriptor[2] == NTN_OTA_PRODUCT_ID && descriptor[3] == NTN_OTA_LAYOUT_ID &&
           descriptor[4] <= NTN_OTA_HW_REV && NTN_OTA_HW_REV <= descriptor[5] &&
           descriptor[6] <= NTN_OTA_BOOT_ABI &&
           (vector[0] & 7U) == 0 && vector[0] > 0x1FFF8018U &&
           vector[0] <= 0x20027000U && (vector[1] & 1U) &&
           entry >= NTN_OTA_APP_VECTOR && entry < NTN_OTA_APP_VECTOR + header->ih_img_size;
}

static uint32_t image_extent(const struct flash_area *area, struct image_header *hdr)
{
    struct image_tlv_info tlv;
    uint32_t off;

    if (flash_area_read(area, 0, hdr, sizeof(*hdr)) || !compatible(area, hdr) ||
        hdr->ih_protect_tlv_size > NTN_OTA_TLV_MAX)
    {
        ntn_ota_log_error("DOWNLOAD_HEADER_OR_TARGET");
        return 0;
    }
    off = NTN_OTA_HEADER_SIZE + hdr->ih_img_size + hdr->ih_protect_tlv_size;
    if (off > NTN_OTA_PACKAGE_MAX ||
        flash_area_read(area, off, &tlv, sizeof(tlv)) ||
        tlv.it_magic != IMAGE_TLV_INFO_MAGIC || tlv.it_tlv_tot < sizeof(tlv) ||
        tlv.it_tlv_tot > NTN_OTA_TLV_MAX - hdr->ih_protect_tlv_size ||
        tlv.it_tlv_tot > NTN_OTA_PACKAGE_MAX - off)
    {
        ntn_ota_log_error("DOWNLOAD_TLV_LENGTH");
        return 0;
    }
    return off + tlv.it_tlv_tot;
}

bool ntn_ota_check_image(const struct flash_area *area, uint32_t received)
{
    static uint8_t work[1024];
    static struct boot_loader_state state;
    static struct boot_sector_buffer sectors;
    struct image_header hdr;
    FIH_DECLARE(result, FIH_FAILURE);
    uint32_t extent = image_extent(area, &hdr);

    if (extent == 0)
        return false;
    if (extent != received)
    {
        ntn_ota_log_error("DOWNLOAD_LENGTH_MISMATCH");
        return false;
    }
    /* Scratch-mode validation uses the sector geometry to bound the image. */
    memset(&state, 0, sizeof(state));
    if (boot_open_all_flash_areas(&state))
    {
        ntn_ota_log_error("DOWNLOAD_FLASH_MAP");
        return false;
    }
    if (boot_read_sectors(&state, &sectors))
    {
        ntn_ota_log_error("DOWNLOAD_SECTOR_GEOMETRY");
    }
    else
    {
        FIH_CALL(bootutil_img_validate, result, &state, &hdr, area, work, sizeof(work), NULL, 0, NULL);
        if (!FIH_EQ(result, FIH_SUCCESS))
            ntn_ota_log_error("DOWNLOAD_HASH_OR_SIGNATURE");
    }
    boot_close_all_flash_areas(&state);
    return FIH_EQ(result, FIH_SUCCESS);
}

/* After boot_go has completed or rejected existing swap work, recover from
 * an independent signed factory image through the normal MCUboot swap path. */
static bool stage_factory_image(void)
{
    static uint8_t sector_data[1024];
    const struct flash_area *factory, *secondary;
    struct image_header hdr;
    uint32_t size;

    if (boot_swap_type() != BOOT_SWAP_TYPE_NONE ||
        flash_area_open(NTN_AREA_FACTORY, &factory) ||
        flash_area_open(FLASH_AREA_IMAGE_SECONDARY(0), &secondary))
        return false;

    size = image_extent(factory, &hdr);
    if (!size || !ntn_ota_check_image(factory, size))
        return false;
    if (flash_area_erase(secondary, 0, NTN_OTA_SLOT_SIZE))
        return false;
    for (uint32_t offset = 0; offset < size;)
    {
        ntn_ota_feed_watchdog();
        uint32_t count = size - offset < sizeof(sector_data) ? size - offset : sizeof(sector_data);
        if (flash_area_read(factory, offset, sector_data, count) ||
            flash_area_write(secondary, offset, sector_data, count))
            return false;
        offset += count;
    }
    return ntn_ota_check_image(secondary, size) && boot_set_pending(1) == 0;
}

int boot_read_image_header_hook(int image, int slot, struct image_header *hdr)
{
    (void)image;
    (void)slot;
    (void)hdr;
    return BOOT_HOOK_REGULAR;
}

fih_ret boot_image_check_hook(int image, int slot)
{
    const struct flash_area *area;
    struct image_header header;
    if (image != 0 || slot < 0 || slot > 1 ||
        flash_area_open((uint8_t)flash_area_id_from_image_slot(slot), &area))
        FIH_RET(FIH_FAILURE);
    bool valid = !flash_area_read(area, 0, &header, sizeof(header)) && compatible(area, &header);
    flash_area_close(area);
    if (!valid)
        FIH_RET(FIH_FAILURE);
    FIH_RET(FIH_BOOT_HOOK_REGULAR);
}

int boot_perform_update_hook(int image, struct image_header *hdr, const struct flash_area *area)
{
    (void)image;
    (void)hdr;
    (void)area;
    return BOOT_HOOK_REGULAR;
}

int boot_read_swap_state_primary_slot_hook(int image, struct boot_swap_state *state)
{
    (void)image;
    (void)state;
    return BOOT_HOOK_REGULAR;
}

int boot_copy_region_post_hook(int image, const struct flash_area *area, size_t size)
{
    (void)image;
    (void)area;
    (void)size;
    return 0;
}

/* The Bootloader only verifies signatures. Entropy-dependent ECC operations
 * must fail until a platform entropy source is installed. */
int default_CSPRNG(uint8_t *dest, unsigned int size)
{
    (void)dest;
    (void)size;
    return 0;
}

static void handoff(void)
{
    unsigned i;
    __disable_irq();
    LL_PERIPH_WE(LL_PERIPH_INTC | LL_PERIPH_EFM | LL_PERIPH_FCG);
    (void)USART_DeInit(CM_USART1);
    (void)QSPI_DeInit();
    /* The HC32F460's unbound selector value is 0x1FF. */
    for (i = 0; i <= (unsigned)INT127_IRQn; ++i)
        (void)INTC_IrqSignOut((IRQn_Type)i);
    CLK_SetSysClockSrc(CLK_SYSCLK_SRC_HRC);
    (void)CLK_PLLCmd(DISABLE);
    EFM_REG_Lock();
    ntn_ota_jump(NTN_OTA_APP_VECTOR);
}

int main(void)
{
    struct boot_rsp response;
    FIH_DECLARE(result, FIH_FAILURE);
    int request = ntn_ota_take_boot_request();
    ntn_ota_feed_watchdog();
    if (app_flash_init() == RT_EOK)
    {
        FIH_CALL(boot_go, result, &response);
        if (FIH_EQ(result, FIH_SUCCESS) && response.br_flash_dev_id == NTN_FLASH_INTERNAL &&
            response.br_image_off == NTN_OTA_PRIMARY &&
            response.br_hdr->ih_hdr_size == NTN_OTA_HEADER_SIZE &&
            ntn_ota_vector_valid(NTN_OTA_APP_VECTOR,
                                 NTN_OTA_APP_VECTOR + response.br_hdr->ih_img_size))
        {
            /* An unconfirmed trial still owns the secondary slot for revert. */
            struct boot_swap_state state;
            if (!boot_read_swap_state_by_id(FLASH_AREA_IMAGE_PRIMARY(0), &state) &&
                state.image_ok == BOOT_FLAG_SET &&
                ntn_ota_serial(request ? 10000U : 1000U,
                               "BOOT2 INFO BL1_OK NOR_OK APP_VERIFIED\r\n"))
                NVIC_SystemReset();
            handoff();
        }
        ntn_ota_report_boot_error("BOOT2 ERROR APP_OR_SWAP\r\n");
        if (stage_factory_image())
        {
            ntn_ota_report_boot_error("BOOT2 INFO FACTORY_STAGED\r\n");
            NVIC_SystemReset();
        }
        while (1)
        {
            if (ntn_ota_serial(10000U, "BOOT2 INFO BL1_OK NOR_OK RECOVERY\r\n"))
                NVIC_SystemReset();
            ntn_ota_feed_watchdog();
            rt_thread_mdelay(10);
        }
    }
    ntn_ota_report_boot_error("BOOT2 ERROR NOR_INIT\r\n");
    for (;;)
        rt_thread_mdelay(1000);
}
